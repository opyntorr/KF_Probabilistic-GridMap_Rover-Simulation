#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class PoseCheck(Node):
    """Print position and yaw from /odometry/filtered without flying."""

    def __init__(self):
        super().__init__('pose_check')
        self.create_subscription(Odometry, '/odometry/filtered', self._cb, _SENSOR_QOS)
        self.get_logger().info('Listening on /odometry/filtered — move the drone by hand to check orientation.')

    def _cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        yaw_deg = math.degrees(yaw)

        warn = ''
        if abs(yaw_deg) > 10.0:
            warn = f'  *** YAW OFFSET — frame rotation will mix X/Y commands ***'

        self.get_logger().info(
            f'pos=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f}) m  '
            f'yaw={yaw_deg:+.1f} deg  '
            f'quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})'
            f'{warn}',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
