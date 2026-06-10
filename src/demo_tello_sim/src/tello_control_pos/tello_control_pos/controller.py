import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64
import math
from rclpy.signals import SignalHandlerOptions

class TelloPositionController(Node):
    def __init__(self):
        super().__init__('tello_position_controller')

        # Publicadores y Suscriptores
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.target_sub = self.create_subscription(Point, '/drone1/target_position', self.target_callback, 10)
        self.land_sub = self.create_subscription(Empty, '/land', self._safety_stop, 1)
        self.emergency_sub = self.create_subscription(Empty, '/emergency', self._safety_stop, 1)

        # Publicadores de diagnóstico — PlotJuggler
        self._dbg_err_pub  = self.create_publisher(Vector3Stamped, '/debug/ctrl/error',    10)
        self._dbg_p_pub    = self.create_publisher(Vector3Stamped, '/debug/ctrl/pid_p',    10)
        self._dbg_i_pub    = self.create_publisher(Vector3Stamped, '/debug/ctrl/pid_i',    10)
        self._dbg_d_pub    = self.create_publisher(Vector3Stamped, '/debug/ctrl/pid_d',    10)
        self._dbg_raw_pub  = self.create_publisher(Vector3Stamped, '/debug/ctrl/cmd_raw',  10)
        self._dbg_sent_pub = self.create_publisher(Vector3Stamped, '/debug/ctrl/cmd_sent', 10)
        self._dbg_dist_pub = self.create_publisher(Float64,        '/debug/ctrl/distance', 10)
        self._dbg_dt_pub   = self.create_publisher(Float64,        '/debug/ctrl/dt',       10)

        self.active = True
        
        # Posición objetivo inicializada en None (esperando comando)
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.target_received = False
        self.has_taken_off = False
        
        # Ganancias del controlador PID (Configurables)
        self.declare_parameter('kp', 2.5)
        self.declare_parameter('ki', 0.8)
        self.declare_parameter('kd', 0.0)
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        
        # Límites de saturación
        self.max_vel = 0.6          # Velocidad máxima normal
        self.max_vel_safety = 1.0   # Techo de velocidad en modo seguridad
        self.safety_gain = 2.0      # Cuánto crece la velocidad por metro² fuera del límite
        self.max_integral = 1.0     # Límite de acumulación

        # Zona de vuelo segura
        self.limit_xy = 2.0
        self.limit_z_min = 0.4
        self.limit_z_max = 3.0

        self.in_safety_mode = False
        
        # Escala de velocidad (1.0 para Gazebo, 100.0 para Tello real)
        self.declare_parameter('velocity_scale', 1.0)
        self.vel_scale = self.get_parameter('velocity_scale').get_parameter_value().double_value
        
        # Memoria de estado para derivadas e integrales
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        
        # Filtro para la derivada (EMA)
        self.filtered_dx = 0.0
        self.filtered_dy = 0.0
        self.filtered_dz = 0.0
        
        self.current_pose = None
        self.current_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.last_time = None
        self.start_time = None
        
        # Watchdog: tiempo de la última odometría integrada recibida
        self._last_integrated_time = 0.0
        self.watchdog_timer = self.create_timer(1.0, self._watchdog)

        # Bucle de control a 100 Hz (sincronizado con OptiTrack a 120 Hz)
        self.timer = self.create_timer(1.0/100.0, self.control_loop)
        
    def _watchdog(self):
        """Warn when /odometry/filtered has not published for too long."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_integrated_time > 0.0 and now - self._last_integrated_time > 0.5:
            self.get_logger().warn(
                f"[ctrl] ADVERTENCIA: sin datos de /odometry/filtered en {now - self._last_integrated_time:.2f} s"
            )

    def _pub_v3(self, pub, x, y, z, stamp):
        """Publish a Vector3Stamped diagnostic message."""
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.vector.x = float(x)
        msg.vector.y = float(y)
        msg.vector.z = float(z)
        pub.publish(msg)

    def _safety_stop(self, msg):
        self.active = False
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().warn("Land/Emergency recibido — controlador silenciado")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        self._last_integrated_time = self.get_clock().now().nanoseconds / 1e9
        
        
    def target_callback(self, msg):
        x = max(min(msg.x,  1.8), -1.8)
        y = max(min(msg.y,  1.8), -1.8)
        z = max(min(msg.z,  2.5),  0.3)

        if x != msg.x or y != msg.y or z != msg.z:
            self.get_logger().warn(
                f"Objetivo fuera de límites ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}) "
                f"— ajustado a ({x:.2f}, {y:.2f}, {z:.2f})"
            )

        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_received = True
        self.get_logger().info(f"Nuevo objetivo: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")
        
    def control_loop(self):
        if not self.active or self.current_pose is None:
            return
            
        current_time = self.get_clock().now()
            
        # Seguridad: no controlar hasta que el dron haya superado la altura de despegue
        if not self.has_taken_off:
            if self.current_pose.z < 0.8:
                self.get_logger().info("Esperando despegue (Z < 0.8m)...", throttle_duration_sec=2.0)
                return
            self.has_taken_off = True
            self.get_logger().info("Despegue detectado. Iniciando control de posicion.")

        # Capturar la posición actual como meta (Hover automático) si aún no hemos recibido un objetivo manual
        if not self.target_received:
            self.target_x = self.current_pose.x
            self.target_y = self.current_pose.y
            self.target_z = 2.5
            self.target_received = True
            self.get_logger().info(f"¡Hover automático activado tras deriva! Anclando a: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")
            
        # Inicializar el tiempo en la primera ejecución activa
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
            
        # --- ZONA SEGURA: velocidad máxima escala cuadráticamente fuera del límite ---
        p = self.current_pose
        overshoot_x = max(0.0, abs(p.x) - self.limit_xy)
        overshoot_y = max(0.0, abs(p.y) - self.limit_xy)
        overshoot_z = max(0.0, p.z - self.limit_z_max) + max(0.0, self.limit_z_min - p.z)
        overshoot = math.sqrt(overshoot_x**2 + overshoot_y**2 + overshoot_z**2)

        active_max_vel = min(self.max_vel + self.safety_gain * overshoot**2, self.max_vel_safety)

        outside = overshoot > 0.0
        if outside:
            if not self.in_safety_mode:
                self.in_safety_mode = True
                self.integral_x = 0.0
                self.integral_y = 0.0
                self.integral_z = 0.0
                self.get_logger().warn(
                    f"¡FUERA DE ZONA SEGURA! ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) "
                    f"exceso={overshoot:.2f}m — retorno cuadrático activado",
                    throttle_duration_sec=0.5
                )
            target_x = max(min(p.x,  self.limit_xy),   -self.limit_xy)
            target_y = max(min(p.y,  self.limit_xy),   -self.limit_xy)
            target_z = max(min(p.z,  self.limit_z_max), self.limit_z_min)
        else:
            if self.in_safety_mode:
                self.in_safety_mode = False
                self.get_logger().info("Dron de vuelta en zona segura — control normal reanudado")
            target_x = self.target_x
            target_y = self.target_y
            target_z = self.target_z

        # 1. Calcular Errores actuales (P)
        error_x = target_x - self.current_pose.x
        error_y = target_y - self.current_pose.y
        error_z = target_z - self.current_pose.z

        # Distancia euclidiana al objetivo
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)

        # Valores PID por defecto (cero cuando el objetivo ya está alcanzado)
        p_x = p_y = p_z = 0.0
        i_x = i_y = i_z = 0.0
        d_x = d_y = d_z = 0.0
        global_vel_x = global_vel_y = vel_z = 0.0

        twist = Twist()

        # Tolerancia para detenerse (15 cm para evitar salir y entrar por ruido)
        if distance > 0.15:

            # 2. Calcular Integrales (I) y aplicar Anti-windup
            self.integral_x += error_x * dt
            self.integral_y += error_y * dt
            self.integral_z += error_z * dt

            self.integral_x = max(min(self.integral_x, self.max_integral), -self.max_integral)
            self.integral_y = max(min(self.integral_y, self.max_integral), -self.max_integral)
            self.integral_z = max(min(self.integral_z, self.max_integral), -self.max_integral)

            # 3. Calcular Derivadas (D)
            raw_dx = (error_x - self.prev_error_x) / dt
            raw_dy = (error_y - self.prev_error_y) / dt
            raw_dz = (error_z - self.prev_error_z) / dt

            # Suavizar derivada (EMA) — alpha bajo para absorber spikes de sensores reales
            alpha_d = 0.2;
            alpha_dz = 0;
            self.filtered_dx = (alpha_d * raw_dx) + ((1.0 - alpha_d) * self.filtered_dx)
            self.filtered_dy = (alpha_d * raw_dy) + ((1.0 - alpha_d) * self.filtered_dy)
            self.filtered_dz = (alpha_dz * raw_dz) + ((1.0 - alpha_dz) * self.filtered_dz)

            # 4. Términos PID separados para diagnóstico
            p_x = self.kp * error_x
            p_y = self.kp * error_y
            p_z = self.kp * error_z

            i_x = self.ki * self.integral_x
            i_y = self.ki * self.integral_y
            i_z = self.ki * self.integral_z

            d_x = self.kd * self.filtered_dx
            d_y = self.kd * self.filtered_dy
            d_z = self.kd * self.filtered_dz

            global_vel_x = p_x + i_x + d_x
            global_vel_y = p_y + i_y + d_y
            vel_z = p_z + i_z + d_z

            # --- ROTACIÓN DE MARCOS: DEL GLOBAL AL LOCAL ---
            q = self.current_orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            local_vel_x = global_vel_x * math.cos(yaw) + global_vel_y * math.sin(yaw)
            local_vel_y = -global_vel_x * math.sin(yaw) + global_vel_y * math.cos(yaw)

            # 5. Aplicar saturación — límite crece cuadráticamente si está fuera de zona
            twist.linear.x = max(min(local_vel_x, active_max_vel), -active_max_vel)
            twist.linear.y = max(min(local_vel_y, active_max_vel), -active_max_vel)
            twist.linear.z = max(min(vel_z,        active_max_vel), -active_max_vel)

        else:
            self.integral_x = 0.0
            self.integral_y = 0.0
            self.integral_z = 0.0
            self.get_logger().info("Posición objetivo alcanzada de forma estable", throttle_duration_sec=2.0)
            
        # Guardar estado para la siguiente iteración
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z
        self.last_time = current_time
            
        # Escalar velocidades para el driver si es necesario
        twist.linear.x *= self.vel_scale
        twist.linear.y *= self.vel_scale
        twist.linear.z *= self.vel_scale

        # Publicar el comando de velocidad
        self.cmd_vel_pub.publish(twist)

        # --- Publicar diagnósticos (PlotJuggler) ---
        now_stamp = self.get_clock().now().to_msg()

        self._pub_v3(self._dbg_err_pub,  error_x, error_y, error_z, now_stamp)
        self._pub_v3(self._dbg_p_pub,    p_x,     p_y,     p_z,     now_stamp)
        self._pub_v3(self._dbg_i_pub,    i_x,     i_y,     i_z,     now_stamp)
        self._pub_v3(self._dbg_d_pub,    d_x,     d_y,     d_z,     now_stamp)
        self._pub_v3(self._dbg_raw_pub,  global_vel_x, global_vel_y, vel_z, now_stamp)
        
        # cmd_sent en m/s físicos (sin factor de escala del driver)
        self._pub_v3(
            self._dbg_sent_pub,
            twist.linear.x / self.vel_scale,
            twist.linear.y / self.vel_scale,
            twist.linear.z / self.vel_scale,
            now_stamp,
        )

        dist_msg = Float64()
        dist_msg.data = distance
        self._dbg_dist_pub.publish(dist_msg)

        dt_msg = Float64()
        dt_msg.data = dt
        self._dbg_dt_pub.publish(dt_msg)

        # --- Logs de texto (throttled 1 s) ---
        self.get_logger().info(
            f'[ctrl] pos=({p.x:.3f},{p.y:.3f},{p.z:.3f})'
            f' objetivo=({self.target_x:.3f},{self.target_y:.3f},{self.target_z:.3f})'
            f' distancia={distance:.3f} m',
            throttle_duration_sec=1.0,
        )
        self.get_logger().info(
            f'[ctrl] dt={dt:.3f} s'
            f' | P=({p_x:.3f},{p_y:.3f},{p_z:.3f})'
            f' I=({i_x:.3f},{i_y:.3f},{i_z:.3f})'
            f' D=({d_x:.3f},{d_y:.3f},{d_z:.3f})'
            f' | vel_bruta=({global_vel_x:.3f},{global_vel_y:.3f},{vel_z:.3f})'
            f' | cmd=({twist.linear.x:.3f},{twist.linear.y:.3f},{twist.linear.z:.3f})',
            throttle_duration_sec=1.0,
        )

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TelloPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo controlador...")
    finally:
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
