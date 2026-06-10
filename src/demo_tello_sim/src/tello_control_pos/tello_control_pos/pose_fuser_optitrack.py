#!/usr/bin/env python3
"""
PoseFuser OptiTrack-first con fallback robusto.

Estrategia:
  1. Primario: pose+orientacion de OptiTrack (/drone_pose, geometry_msgs/PoseStamped).
  2. Mientras OptiTrack viva, mantiene un offset SE(3) T_world_odom calculado
     como T_world_drone_opti * inv(T_odom_drone). El offset captura la deriva
     instantanea del odom interno del Tello.
  3. Si OptiTrack se ausenta brevemente (< opti_fresh_window_sec + extrap_window_sec),
     extrapola la posicion con la ultima velocidad lineal estimada (orientacion
     congelada en el ultimo valor bueno).
  4. Si el hueco supera la extrapolacion, publica T_world_odom * T_odom_drone:
     el offset garantiza que NO hay salto al cambiar de fuente.
  5. Al recuperar OptiTrack, blending con smoothstep en posicion y SLERP en
     orientacion durante blend_duration_sec.
  6. Rechazo de outliers de OptiTrack por salto excesivo de translacion/rotacion
     vs la ultima muestra valida; tras max_consecutive_rejections seguidos,
     acepta la muestra (presunto teleport real / reset de OptiTrack).

Publicaciones:
  - /odometry/filtered (nav_msgs/Odometry)
  - /odometry/source   (std_msgs/String): "optitrack" | "blending" |
                       "extrapolation" | "odom_offset" | "none"

Correccion de flip de OptiTrack:
  Cuando el rigid body se define con 4 marcadores coplanares, el solver puede
  devolver la solucion 'volteada' (rotacion de 180 deg en eje horizontal).
  Detectada por (a) componente Z mundo del eje +Z cuerpo, y (b) salto angular
  grande respecto a la ultima cuaterna buena. Se corrige multiplicando por
  R_x(180 deg) en el cuerpo, conservando el yaw.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ---------- Cuaternas (x, y, z, w) ----------

def _quat_tuple(q: Quaternion):
    return (q.x, q.y, q.z, q.w)


def _quat_normalize(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / n, qy / n, qz / n, qw / n)


def _quat_mul(a, b):
    """Producto Hamilton, ambos (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_rotate_vec(q, v):
    """Rotacion de un vector 3D por una cuaterna unitaria."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _world_z_of_body_z(qx, qy, qz, qw):
    return 1.0 - 2.0 * (qx * qx + qy * qy)


def _angular_distance(q1, q2):
    dot = abs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _quat_slerp(q1, q2, t):
    dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]
    if dot < 0.0:
        q2 = (-q2[0], -q2[1], -q2[2], -q2[3])
        dot = -dot
    if dot > 0.9995:
        return _quat_normalize(
            q1[0] + t * (q2[0] - q1[0]),
            q1[1] + t * (q2[1] - q1[1]),
            q1[2] + t * (q2[2] - q1[2]),
            q1[3] + t * (q2[3] - q1[3]),
        )
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    a = math.sin((1.0 - t) * theta) / sin_theta
    b = math.sin(t * theta) / sin_theta
    return _quat_normalize(
        a * q1[0] + b * q2[0],
        a * q1[1] + b * q2[1],
        a * q1[2] + b * q2[2],
        a * q1[3] + b * q2[3],
    )


# ---------- SE(3) ----------

def _se3_compose(ta, qa, tb, qb):
    """T_ac = T_ab * T_bc. Devuelve (t_ac, q_ac)."""
    q_new = _quat_mul(qa, qb)
    t_rot = _quat_rotate_vec(qa, tb)
    return (ta[0] + t_rot[0], ta[1] + t_rot[1], ta[2] + t_rot[2]), q_new


def _se3_invert(t, q):
    q_inv = _quat_conj(q)
    t_inv = _quat_rotate_vec(q_inv, t)
    return (-t_inv[0], -t_inv[1], -t_inv[2]), q_inv


# Rotacion 180 deg en eje X del cuerpo (corrige flip de OptiTrack)
_FLIP_X = (1.0, 0.0, 0.0, 0.0)


def _apply_flip_correction(q):
    return _quat_normalize(*_quat_mul(q, _FLIP_X))


# ---------- Nodo ----------

class PoseFuserOptitrack(Node):
    SRC_OPTI = 'optitrack'
    SRC_BLEND = 'blending'
    SRC_EXTRAP = 'extrapolation'
    SRC_ODOM_OFFSET = 'odom_offset'
    SRC_NONE = 'none'

    def __init__(self):
        super().__init__('pose_fuser_optitrack')

        # --- Parametros temporales (auto-adaptativos al rate observado) ---
        # En lugar de un timeout fijo, el fuser mide el intervalo medio entre
        # muestras de OptiTrack y ajusta sus ventanas en proporcion:
        #   fresh  = clamp(fresh_factor  * intervalo_observado, fresh_min,  fresh_max)
        #   extrap = clamp(extrap_factor * intervalo_observado, extrap_min, extrap_max)
        # Asi el mismo binario sirve para OptiTrack real (~120 Hz, ~8 ms) y para
        # simulador/odom lento (~4 Hz, ~270 ms) sin tocar parametros.
        self.declare_parameter('fresh_factor', 3.0)
        self.declare_parameter('fresh_min_sec', 0.05)
        self.declare_parameter('fresh_max_sec', 1.50)
        self.declare_parameter('extrap_factor', 1.5)
        self.declare_parameter('extrap_min_sec', 0.05)
        self.declare_parameter('extrap_max_sec', 1.00)
        self.declare_parameter('interval_filter_alpha', 0.30)
        self.declare_parameter('odom_max_stale_sec', 0.50)
        self.declare_parameter('blend_duration_sec', 0.30)
        self.declare_parameter('publish_rate_hz', 100.0)

        # --- Validacion de OptiTrack ---
        self.declare_parameter('max_jump_translation_m', 0.40)
        self.declare_parameter('max_jump_rotation_deg', 45.0)
        self.declare_parameter('max_consecutive_rejections', 5)
        self.declare_parameter('outlier_check_max_dt', 0.10)

        # --- Estimacion de velocidad para extrapolacion ---
        self.declare_parameter('vel_filter_alpha', 0.30)
        self.declare_parameter('min_dt_for_vel', 0.005)

        # --- Correccion de flip ---
        self.declare_parameter('flip_jump_thresh_deg', 90.0)
        self.declare_parameter('flip_zaxis_thresh', 0.0)

        gp = lambda n: self.get_parameter(n).get_parameter_value()
        self.fresh_factor = gp('fresh_factor').double_value
        self.fresh_min = gp('fresh_min_sec').double_value
        self.fresh_max = gp('fresh_max_sec').double_value
        self.extrap_factor = gp('extrap_factor').double_value
        self.extrap_min = gp('extrap_min_sec').double_value
        self.extrap_max = gp('extrap_max_sec').double_value
        self.interval_alpha = gp('interval_filter_alpha').double_value
        self.odom_max_stale = gp('odom_max_stale_sec').double_value
        self.blend_duration = gp('blend_duration_sec').double_value
        rate = gp('publish_rate_hz').double_value
        # Intervalo observado entre muestras de OptiTrack (EMA). 0 hasta primer dato.
        self.observed_interval = 0.0
        self.max_jump_t = gp('max_jump_translation_m').double_value
        self.max_jump_q = math.radians(gp('max_jump_rotation_deg').double_value)
        self.max_consec_rej = gp('max_consecutive_rejections').integer_value
        self.outlier_max_dt = gp('outlier_check_max_dt').double_value
        self.vel_alpha = gp('vel_filter_alpha').double_value
        self.min_dt_vel = gp('min_dt_for_vel').double_value
        self.jump_thresh = math.radians(gp('flip_jump_thresh_deg').double_value)
        self.z_thresh = gp('flip_zaxis_thresh').double_value

        # --- I/O ---
        self.opti_sub = self.create_subscription(PoseStamped, '/drone_pose', self._opti_cb, _SENSOR_QOS)
        self.odom_sub = self.create_subscription(Odometry, '/drone1/odom', self._odom_cb, 10)
        self.fused_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.source_pub = self.create_publisher(String, '/odometry/source', 10)

        # --- Estado OptiTrack ---
        self.latest_opti_t = None
        self.latest_opti_q = None
        self.last_opti_time = None
        self.opti_velocity = (0.0, 0.0, 0.0)
        self.rejected_count = 0
        self.last_good_quat = None

        # --- Estado odom ---
        self.latest_odom = None
        self.last_odom_time = None

        # --- Offset T_world_odom ---
        self.offset_t = (0.0, 0.0, 0.0)
        self.offset_q = (0.0, 0.0, 0.0, 1.0)
        self.offset_valid = False

        # --- Estado de publicacion ---
        self.last_published_t = None
        self.last_published_q = None
        self.last_raw_source = self.SRC_NONE

        # --- Blending ---
        self.is_blending = False
        self.blend_start_time = None
        self.blend_start_t = None
        self.blend_start_q = None

        # --- Telemetria interna ---
        self.flip_count = 0
        self.warned_no_data = False
        self.last_logged_source = None

        self.timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)

        self.get_logger().info(
            f"PoseFuserOptitrack iniciado (auto-adaptativo): "
            f"fresh = clamp({self.fresh_factor}*dt, [{self.fresh_min:.2f},{self.fresh_max:.2f}])s, "
            f"extrap = clamp({self.extrap_factor}*dt, [{self.extrap_min:.2f},{self.extrap_max:.2f}])s, "
            f"odom_max_stale={self.odom_max_stale:.2f}s, blend={self.blend_duration:.2f}s, "
            f"max_jump=({self.max_jump_t:.2f}m, {math.degrees(self.max_jump_q):.0f}deg)"
        )

    # ---------- Correccion de flip ----------

    def _correct_flip(self, raw_q):
        qx, qy, qz, qw = _quat_normalize(*raw_q)
        q = (qx, qy, qz, qw)

        body_z_world = _world_z_of_body_z(qx, qy, qz, qw)
        need_flip = body_z_world < self.z_thresh

        if not need_flip and self.last_good_quat is not None:
            d_raw = _angular_distance(q, self.last_good_quat)
            if d_raw > self.jump_thresh:
                q_alt = _apply_flip_correction(q)
                d_alt = _angular_distance(q_alt, self.last_good_quat)
                if d_alt < d_raw:
                    need_flip = True

        if need_flip:
            q = _apply_flip_correction(q)
            self.flip_count += 1
            self.get_logger().warn(
                f"OptiTrack: flip detectado — correccion 180 deg aplicada (total={self.flip_count})",
                throttle_duration_sec=1.0,
            )

        self.last_good_quat = q
        return q

    # ---------- Callbacks ----------

    def _opti_cb(self, msg: PoseStamped):
        now = self.get_clock().now()
        raw_q = _quat_tuple(msg.pose.orientation)
        corrected_q = self._correct_flip(raw_q)
        new_t = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        # Rechazo de outliers (solo si tenemos referencia reciente)
        if self.latest_opti_t is not None and self.last_opti_time is not None:
            dt = (now - self.last_opti_time).nanoseconds / 1e9
            if 0.0 < dt < self.outlier_max_dt:
                trans_jump = math.sqrt(
                    (new_t[0] - self.latest_opti_t[0]) ** 2
                    + (new_t[1] - self.latest_opti_t[1]) ** 2
                    + (new_t[2] - self.latest_opti_t[2]) ** 2
                )
                rot_jump = _angular_distance(corrected_q, self.latest_opti_q)
                if trans_jump > self.max_jump_t or rot_jump > self.max_jump_q:
                    self.rejected_count += 1
                    if self.rejected_count <= self.max_consec_rej:
                        self.get_logger().warn(
                            f"OptiTrack outlier rechazado: dt={trans_jump:.3f}m, "
                            f"dq={math.degrees(rot_jump):.1f}deg "
                            f"(consecutivos={self.rejected_count})",
                            throttle_duration_sec=0.5,
                        )
                        return
                    self.get_logger().warn(
                        f"OptiTrack: {self.rejected_count} outliers seguidos — "
                        "aceptando muestra (posible teleport real)"
                    )

            # Estimacion de velocidad (filtro EMA)
            if dt > self.min_dt_vel:
                inst_v = (
                    (new_t[0] - self.latest_opti_t[0]) / dt,
                    (new_t[1] - self.latest_opti_t[1]) / dt,
                    (new_t[2] - self.latest_opti_t[2]) / dt,
                )
                a = self.vel_alpha
                self.opti_velocity = (
                    a * inst_v[0] + (1.0 - a) * self.opti_velocity[0],
                    a * inst_v[1] + (1.0 - a) * self.opti_velocity[1],
                    a * inst_v[2] + (1.0 - a) * self.opti_velocity[2],
                )

            # Estimacion del intervalo entre muestras (EMA). Solo cuenta gaps
            # razonables: ignora gaps enormes (probable restart de OptiTrack)
            # que distorsionarian el filtro.
            if 0.0 < dt < self.fresh_max:
                if self.observed_interval > 0.0:
                    a = self.interval_alpha
                    self.observed_interval = a * dt + (1.0 - a) * self.observed_interval
                else:
                    self.observed_interval = dt

        if self.latest_opti_t is None:
            self.get_logger().info("Primer dato de OptiTrack recibido.")

        self.rejected_count = 0
        self.latest_opti_t = new_t
        self.latest_opti_q = corrected_q
        self.last_opti_time = now

        # Actualizar offset si odom esta fresco
        if self.latest_odom is not None and self.last_odom_time is not None:
            odom_age = (now - self.last_odom_time).nanoseconds / 1e9
            if odom_age < self.odom_max_stale:
                self._update_offset()

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg
        self.last_odom_time = self.get_clock().now()

    # ---------- Offset SE(3) ----------

    def _update_offset(self):
        """T_world_odom = T_world_drone * inv(T_odom_drone)."""
        t_wd = self.latest_opti_t
        q_wd = self.latest_opti_q
        op = self.latest_odom.pose.pose
        t_od = (op.position.x, op.position.y, op.position.z)
        q_od = _quat_tuple(op.orientation)
        t_do, q_do = _se3_invert(t_od, q_od)
        self.offset_t, self.offset_q = _se3_compose(t_wd, q_wd, t_do, q_do)
        if not self.offset_valid:
            self.get_logger().info(
                f"Offset T_world_odom inicializado: t=({self.offset_t[0]:.3f}, "
                f"{self.offset_t[1]:.3f}, {self.offset_t[2]:.3f})"
            )
        self.offset_valid = True

    def _apply_offset_to_odom(self):
        if not self.offset_valid or self.latest_odom is None:
            return None
        op = self.latest_odom.pose.pose
        t_od = (op.position.x, op.position.y, op.position.z)
        q_od = _quat_tuple(op.orientation)
        return _se3_compose(self.offset_t, self.offset_q, t_od, q_od)

    # ---------- Ventanas adaptativas ----------

    def _effective_windows(self):
        """Calcula (fresh, extrap) escaladas al intervalo observado, con clamps.
        Hasta tener una observacion usa los limites inferiores."""
        if self.observed_interval <= 0.0:
            return self.fresh_min, self.extrap_min
        fresh = max(self.fresh_min,
                    min(self.fresh_max,
                        self.fresh_factor * self.observed_interval))
        extrap = max(self.extrap_min,
                     min(self.extrap_max,
                         self.extrap_factor * self.observed_interval))
        return fresh, extrap

    # ---------- Decision de pose cruda ----------

    def _compute_raw_pose(self, now):
        """Devuelve ((t, q), source). Si no hay datos: (None, 'none')."""
        dt_opti = None
        if self.last_opti_time is not None:
            dt_opti = (now - self.last_opti_time).nanoseconds / 1e9

        fresh, extrap = self._effective_windows()

        if (dt_opti is not None
                and dt_opti < fresh
                and self.latest_opti_t is not None):
            return (self.latest_opti_t, self.latest_opti_q), self.SRC_OPTI

        if (dt_opti is not None
                and dt_opti < fresh + extrap
                and self.latest_opti_t is not None):
            t_ex = (
                self.latest_opti_t[0] + self.opti_velocity[0] * dt_opti,
                self.latest_opti_t[1] + self.opti_velocity[1] * dt_opti,
                self.latest_opti_t[2] + self.opti_velocity[2] * dt_opti,
            )
            return (t_ex, self.latest_opti_q), self.SRC_EXTRAP

        if (self.offset_valid
                and self.latest_odom is not None
                and self.last_odom_time is not None):
            dt_odom = (now - self.last_odom_time).nanoseconds / 1e9
            if dt_odom < self.odom_max_stale:
                pose = self._apply_offset_to_odom()
                if pose is not None:
                    return pose, self.SRC_ODOM_OFFSET

        return None, self.SRC_NONE

    # ---------- Publish ----------

    def _publish(self):
        now = self.get_clock().now()
        raw_pose, raw_src = self._compute_raw_pose(now)

        if raw_src == self.SRC_NONE:
            if not self.warned_no_data:
                self.get_logger().warn("Sin datos validos (OptiTrack y odom) — esperando…")
                self.warned_no_data = True
            self._publish_source(self.SRC_NONE)
            self.last_raw_source = raw_src
            return
        self.warned_no_data = False

        # Transicion fallback -> opti: arrancar blending SOLO si veniamos de
        # odom_offset (es ahi donde puede haber un salto real). De extrap a opti
        # la pose es continua (extrap = ultima opti + v*dt), no necesita blending
        # y evita falsos arranques cada muestra cuando el emisor es lento.
        was_hard_fallback = self.last_raw_source == self.SRC_ODOM_OFFSET
        if (was_hard_fallback
                and raw_src == self.SRC_OPTI
                and self.last_published_t is not None
                and self.blend_duration > 0.0):
            self.blend_start_time = now
            self.blend_start_t = self.last_published_t
            self.blend_start_q = self.last_published_q
            self.is_blending = True
            self.get_logger().info(
                f"OptiTrack recuperado tras '{self.last_raw_source}' — "
                f"blending {self.blend_duration:.2f}s"
            )

        published_src = raw_src
        pose = raw_pose
        if self.is_blending:
            blend_dt = (now - self.blend_start_time).nanoseconds / 1e9
            # Continuar blending tambien sobre extrap: si el emisor publica a
            # ritmo lento (e.g. simulador a 20Hz) cada muestra alterna opti/extrap
            # y abortar abruptamente generaria flapping.
            cont = raw_src in (self.SRC_OPTI, self.SRC_EXTRAP)
            if blend_dt < self.blend_duration and cont:
                u = max(0.0, min(1.0, blend_dt / self.blend_duration))
                u = u * u * (3.0 - 2.0 * u)  # smoothstep
                t_a, q_a = self.blend_start_t, self.blend_start_q
                t_b, q_b = pose
                t_blend = (
                    t_a[0] + u * (t_b[0] - t_a[0]),
                    t_a[1] + u * (t_b[1] - t_a[1]),
                    t_a[2] + u * (t_b[2] - t_a[2]),
                )
                q_blend = _quat_slerp(q_a, q_b, u)
                pose = (t_blend, q_blend)
                published_src = self.SRC_BLEND
            else:
                self.is_blending = False
                if cont:
                    self.get_logger().info("Blending completado.")

        self._publish_odometry(now, pose)
        self._publish_source(published_src)

        self.last_published_t, self.last_published_q = pose
        self.last_raw_source = raw_src

        if published_src != self.last_logged_source:
            fresh, extrap = self._effective_windows()
            rate_hz = (1.0 / self.observed_interval) if self.observed_interval > 0 else 0.0
            self.get_logger().info(
                f"Fuente activa: {published_src} "
                f"(opti @ {rate_hz:.1f} Hz, fresh={fresh*1000:.0f}ms, extrap={extrap*1000:.0f}ms)",
                throttle_duration_sec=1.0,
            )
            self.last_logged_source = published_src

    def _publish_odometry(self, now, pose):
        t, q = pose
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = t[0]
        msg.pose.pose.position.y = t[1]
        msg.pose.pose.position.z = t[2]
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        if self.latest_odom is not None:
            msg.twist = self.latest_odom.twist
        self.fused_pub.publish(msg)

    def _publish_source(self, src):
        m = String()
        m.data = src
        self.source_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PoseFuserOptitrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
