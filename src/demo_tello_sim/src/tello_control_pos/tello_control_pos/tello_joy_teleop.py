import os
import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import math


class TelloJoyTeleop(Node):
    """
    Teleop del Tello con control Xbox.

    Modo velocidad directa: los sticks publican velocidades a /drone1/cmd_vel.
    Al soltar los sticks (zona muerta), publica velocidad cero para que el dron
    se quede hovereando en su lugar.

    Mapeo Xbox One S (verificado):
      Stick izq vertical  (eje 0)  -> X  adelante/atras
      Stick izq horizontal (eje 1) -> Y  lateral
      Stick der horizontal (eje 2) -> Yaw girar
      LT (eje 4)                   -> bajar
      RT (eje 5)                   -> subir

      Boton A (0) -> Takeoff
      Boton B (1) -> Land
      Boton X (2) -> Emergency stop
    """

    def __init__(self):
        super().__init__('tello_joy_teleop')

        # ---- Parametros configurables ----
        self.declare_parameter('axis_x', 2)
        self.declare_parameter('axis_y', 1)
        self.declare_parameter('axis_z', 4)
        self.declare_parameter('axis_yaw', 3)

        self.declare_parameter('takeoff_button', 0)    # A
        self.declare_parameter('land_button', 1)        # B
        self.declare_parameter('emergency_button', 2)   # X

        # Bumpers para yaw (-1 = deshabilitado, usa axis_yaw en su lugar)
        self.declare_parameter('yaw_left_button', -1)   # LB = 4
        self.declare_parameter('yaw_right_button', -1)  # RB = 5

        # Gatillos para Z (-1 = deshabilitado, usa axis_z en su lugar)
        # Convencion: gatillo en reposo = 1.0, presionado va hacia -1.0
        # Formula: vz = (rt - lt) / 2  →  0 en reposo, +1 lt presionado, -1 rt presionado
        self.declare_parameter('trigger_z_up_axis', -1)    # LT = subir
        self.declare_parameter('trigger_z_down_axis', -1)  # RT = bajar

        self.declare_parameter('max_linear_vel', 0.5)   # m/s (sim) o unidades/100 (real)
        self.declare_parameter('max_z_vel', 0.4)
        self.declare_parameter('max_yaw_vel', 1.0)      # rad/s
        self.declare_parameter('velocity_scale', 1.0)   # 1.0 sim, 100.0 real
        self.declare_parameter('deadzone', 0.12)         # zona muerta del stick

        # Modo de takeoff/land: 'service' para sim, 'topic' para real
        self.declare_parameter('takeoff_mode', 'service')

        # Foto: -1 = deshabilitado
        self.declare_parameter('photo_button', -1)
        self.declare_parameter('photo_save_dir', '/tmp/tello_fotos')

        # Leer parametros
        self.axis_x = self.get_parameter('axis_x').value
        self.axis_y = self.get_parameter('axis_y').value
        self.axis_z = self.get_parameter('axis_z').value
        self.axis_yaw = self.get_parameter('axis_yaw').value

        self.takeoff_btn = self.get_parameter('takeoff_button').value
        self.land_btn = self.get_parameter('land_button').value
        self.emergency_btn = self.get_parameter('emergency_button').value
        self.yaw_left_btn = self.get_parameter('yaw_left_button').value
        self.yaw_right_btn = self.get_parameter('yaw_right_button').value
        self.use_bumper_yaw = (self.yaw_left_btn >= 0 and self.yaw_right_btn >= 0)

        self.trigger_z_up = self.get_parameter('trigger_z_up_axis').value
        self.trigger_z_down = self.get_parameter('trigger_z_down_axis').value
        self.use_trigger_z = (self.trigger_z_up >= 0 and self.trigger_z_down >= 0)

        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_z = self.get_parameter('max_z_vel').value
        self.max_yaw = self.get_parameter('max_yaw_vel').value
        self.vel_scale = self.get_parameter('velocity_scale').value
        self.deadzone = self.get_parameter('deadzone').value

        self.takeoff_mode = self.get_parameter('takeoff_mode').value
        self.photo_btn = self.get_parameter('photo_button').value
        self.photo_dir = self.get_parameter('photo_save_dir').value

        # ---- Estado interno ----
        self.is_flying = False
        self.emergency_active = False
        self.latest_image: Image | None = None

        # Debounce: guarda el estado anterior de los botones para detectar flancos
        self.prev_buttons = []

        # ---- Pub / Sub ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Topics para dron real
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
        self.land_pub = self.create_publisher(Empty, '/land', 1)
        self.emergency_pub = self.create_publisher(Empty, '/emergency', 1)

        if self.photo_btn >= 0:
            self.create_subscription(Image, '/image_raw', self._image_callback, 1)

        self.get_logger().info('Tello Joy Teleop iniciado')
        self.get_logger().info(f'  Modo takeoff: {self.takeoff_mode}')
        self.get_logger().info(f'  Vel scale: {self.vel_scale}')
        self.get_logger().info(f'  Max vel XY: {self.max_linear} | Z: {self.max_z} | Yaw: {self.max_yaw}')
        self.get_logger().info('  A=takeoff  B=land  X=emergency')
        if self.use_bumper_yaw:
            self.get_logger().info(f'  Yaw: LB(btn {self.yaw_left_btn})=izq  RB(btn {self.yaw_right_btn})=der')
        if self.use_trigger_z:
            self.get_logger().info(f'  Z: LT(eje {self.trigger_z_up})=subir  RT(eje {self.trigger_z_down})=bajar')
        else:
            self.get_logger().info(f'  Z: eje {self.axis_z}')
        if self.photo_btn >= 0:
            self.get_logger().info(f'  Foto: btn Y({self.photo_btn}) → {self.photo_dir}')

    def _button_pressed(self, buttons, index):
        """Detecta flanco de subida (pressed this frame, not last frame)."""
        if index >= len(buttons):
            return False
        current = buttons[index] == 1
        previous = (self.prev_buttons[index] == 1) if index < len(self.prev_buttons) else False
        return current and not previous

    def _apply_deadzone(self, value):
        """Aplica zona muerta simetrica."""
        if abs(value) < self.deadzone:
            return 0.0
        # Re-escalar para que la salida empiece desde 0 despues de la zona muerta
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def joy_callback(self, msg: Joy):
        # ---- Botones (flanco de subida) ----
        if self._button_pressed(msg.buttons, self.takeoff_btn):
            self._do_takeoff()

        if self._button_pressed(msg.buttons, self.land_btn):
            self._do_land()

        if self._button_pressed(msg.buttons, self.emergency_btn):
            self._do_emergency()

        if self.photo_btn >= 0 and self._button_pressed(msg.buttons, self.photo_btn):
            self._take_photo()

        # Guardar estado de botones para el siguiente ciclo
        self.prev_buttons = list(msg.buttons)

        # ---- Sticks -> Velocidad ----
        if self.emergency_active:
            # Silenciar todo en emergencia
            self.cmd_vel_pub.publish(Twist())
            return

        if not self.is_flying:
            return

        twist = Twist()

        # Leer ejes con zona muerta
        raw_x = msg.axes[self.axis_x] if self.axis_x < len(msg.axes) else 0.0
        raw_y = msg.axes[self.axis_y] if self.axis_y < len(msg.axes) else 0.0
        vx = self._apply_deadzone(raw_x)
        vy = self._apply_deadzone(raw_y)

        if self.use_trigger_z:
            up = msg.axes[self.trigger_z_up] if self.trigger_z_up < len(msg.axes) else 1.0
            dn = msg.axes[self.trigger_z_down] if self.trigger_z_down < len(msg.axes) else 1.0
            # Reposo: up=dn=1.0 → vz=0. up presionado (-1.0) → sube, dn presionado (-1.0) → baja
            vz = (dn - up) / 2.0
        else:
            raw_z = msg.axes[self.axis_z] if self.axis_z < len(msg.axes) else 0.0
            vz = self._apply_deadzone(raw_z)

        # Yaw: bumpers (digital) o stick (analogico)
        if self.use_bumper_yaw:
            lb = msg.buttons[self.yaw_left_btn] if self.yaw_left_btn < len(msg.buttons) else 0
            rb = msg.buttons[self.yaw_right_btn] if self.yaw_right_btn < len(msg.buttons) else 0
            vyaw = float(lb) - float(rb)  # LB=+1 (izq), RB=-1 (der)
        else:
            raw_yaw = msg.axes[self.axis_yaw] if self.axis_yaw < len(msg.axes) else 0.0
            vyaw = self._apply_deadzone(raw_yaw)

        # Escalar a velocidad maxima
        twist.linear.x = vx * self.max_linear * self.vel_scale
        twist.linear.y = vy * self.max_linear * self.vel_scale
        twist.linear.z = vz * self.max_z * self.vel_scale
        twist.angular.z = vyaw * self.max_yaw * self.vel_scale

        self.cmd_vel_pub.publish(twist)

    def _do_takeoff(self):
        if self.is_flying:
            self.get_logger().warn('Ya esta volando, ignorando takeoff')
            return

        self.emergency_active = False

        if self.takeoff_mode == 'manual':
            # Manual: solo habilitar sticks, el usuario controla todo
            self.get_logger().info('TAKEOFF (manual) - sticks habilitados, sube con stick derecho')
        elif self.takeoff_mode == 'service':
            # Simulacion: llamar al servicio de Gazebo
            self.get_logger().info('TAKEOFF (service)')
            self._call_tello_service('takeoff')
        else:
            # Real: publicar a /takeoff
            self.get_logger().info('TAKEOFF (topic)')
            self.takeoff_pub.publish(Empty())

        self.is_flying = True

    def _do_land(self):
        if not self.is_flying:
            self.get_logger().warn('No esta volando, ignorando land')
            return

        self.get_logger().info('LAND')

        # Detener velocidad primero
        self.cmd_vel_pub.publish(Twist())

        if self.takeoff_mode == 'service':
            self._call_tello_service('land')
        elif self.takeoff_mode == 'topic':
            self.land_pub.publish(Empty())
        # En modo manual, solo deshabilita sticks (el usuario ya bajo con el stick)

        self.is_flying = False

    def _do_emergency(self):
        self.get_logger().error('EMERGENCY STOP')
        self.emergency_active = True
        self.is_flying = False

        # Detener todo
        self.cmd_vel_pub.publish(Twist())

        if self.takeoff_mode == 'topic':
            self.emergency_pub.publish(Empty())
        else:
            self._call_tello_service('land')

    def _image_callback(self, msg: Image):
        self.latest_image = msg

    def _take_photo(self):
        if self.latest_image is None:
            self.get_logger().warn('Foto: no hay imagen disponible aun')
            return
        try:
            from cv_bridge import CvBridge
            import cv2
            img = CvBridge().imgmsg_to_cv2(self.latest_image, 'bgr8')
            ts = datetime.datetime.now().strftime('%H%M%S_%f')[:-3]
            path = os.path.join(self.photo_dir, f'foto_{ts}.jpg')
            cv2.imwrite(path, img)
            self.get_logger().info(f'Foto guardada: {path}')
        except Exception as e:
            self.get_logger().error(f'Error guardando foto: {e}')

    def _call_tello_service(self, cmd):
        """Llama al servicio de accion del Tello en simulacion.
        Usa un subproceso para no bloquear el callback del joy."""
        import subprocess
        try:
            subprocess.Popen([
                'ros2', 'service', 'call',
                '/drone1/tello_action',
                'tello_msgs/srv/TelloAction',
                f"{{cmd: '{cmd}'}}"
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f'Error llamando servicio tello_action: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TelloJoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo teleop...')
    finally:
        # Parar el dron al salir
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
