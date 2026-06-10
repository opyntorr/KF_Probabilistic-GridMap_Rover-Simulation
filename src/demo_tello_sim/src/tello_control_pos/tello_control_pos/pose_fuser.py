#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

class PoseFuser(Node):
    def __init__(self):
        super().__init__('pose_fuser')

        # Suscriptores
        self.opti_sub = self.create_subscription(
            PoseStamped,
            '/drone_pose',
            self.opti_callback,
            _SENSOR_QOS,
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone1/odom',
            self.odom_callback,
            10
        )
        
        # Publicador
        self.fused_pub = self.create_publisher(
            Odometry,
            '/odometry/filtered',
            10
        )
        
        # Estado interno
        self.latest_opti_pose = None
        self.last_opti_time = None
        self.optitrack_timeout = 0.5  # segundos sin datos antes de activar fallback

        self._last_odom_time = 0.0
        self.watchdog_timer = self.create_timer(1.0, self._watchdog)

        self.get_logger().info("PoseFuser iniciado: Fusionando posición de OptiTrack con orientación de Odometría")

    def _watchdog(self):
        """Warn when a sensor has not published for too long."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_odom_time > 0.0 and now - self._last_odom_time > 0.5:
            self.get_logger().warn(
                f"[fuser] ADVERTENCIA: sin datos de /drone1/odom en {now - self._last_odom_time:.2f} s"
            )

    def opti_callback(self, msg):
        if self.latest_opti_pose is None:
            self.get_logger().info("¡Primer dato de OptiTrack recibido!")
        self.latest_opti_pose = msg.pose.position
        self.last_opti_time = self.get_clock().now()

    def odom_callback(self, msg):
        self._last_odom_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("Dato de odometría del Tello recibido", throttle_duration_sec=2.0)

        now = self.get_clock().now()
        optitrack_alive = (
            self.last_opti_time is not None and
            (now - self.last_opti_time).nanoseconds / 1e9 < self.optitrack_timeout
        )

        fused_msg = Odometry()
        fused_msg.header.stamp = now.to_msg()
        fused_msg.header.frame_id = "odom"
        fused_msg.child_frame_id = "base_link"

        if optitrack_alive:
            # Caso 1 — Fusión completa: posición absoluta de OptiTrack + orientación de odometría
            fused_msg.pose.pose.position.x = self.latest_opti_pose.x
            fused_msg.pose.pose.position.y = self.latest_opti_pose.y
            fused_msg.pose.pose.position.z = self.latest_opti_pose.z
            fused_msg.pose.pose.orientation = msg.pose.pose.orientation
        elif self.latest_opti_pose is not None:
            # Caso 2 — OptiTrack caído: congelar X,Y,Z en última posición conocida
            self.get_logger().warn("OptiTrack timeout — posición congelada en último dato conocido", throttle_duration_sec=1.0)
            fused_msg.pose.pose.position.x = self.latest_opti_pose.x
            fused_msg.pose.pose.position.y = self.latest_opti_pose.y
            fused_msg.pose.pose.position.z = self.latest_opti_pose.z
            fused_msg.pose.pose.orientation = msg.pose.pose.orientation
        else:
            # Sin OptiTrack inicial — el sistema no puede operar de forma segura
            self.get_logger().warn("Esperando primer dato de OptiTrack — sistema bloqueado", throttle_duration_sec=2.0)
            return

        fused_msg.twist = msg.twist
        self.fused_pub.publish(fused_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
