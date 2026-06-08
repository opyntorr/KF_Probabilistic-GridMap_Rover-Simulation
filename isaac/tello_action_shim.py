#!/usr/bin/env python3
# Shim TelloAction: provee el servicio /drone1/tello_action (tello_msgs/TelloAction) que
# usa mision_dron en modo sim, y lo relaya a los topics Empty /drone1/{takeoff,land} que
# consume el dron KINEMÁTICO de Isaac (drone.py). Así la misión del dron de Gazebo corre
# sin cambios sobre Isaac. Correr en el HOST (dominio 30, workspace de Gazebo sourceado).
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from tello_msgs.srv import TelloAction


class TelloActionShim(Node):
    def __init__(self):
        super().__init__("tello_action_shim")
        self._tk = self.create_publisher(Empty, "/drone1/takeoff", 1)
        self._ld = self.create_publisher(Empty, "/drone1/land", 1)
        self.create_service(TelloAction, "/drone1/tello_action", self._cb)
        self.get_logger().info(
            "TelloAction shim listo: /drone1/tello_action -> /drone1/{takeoff,land}")

    def _cb(self, req, resp):
        cmd = (getattr(req, "cmd", "") or "").strip().lower()
        if cmd == "takeoff":
            self._tk.publish(Empty()); self.get_logger().info("takeoff -> /drone1/takeoff")
        elif cmd == "land":
            self._ld.publish(Empty()); self.get_logger().info("land -> /drone1/land")
        else:
            self.get_logger().warn(f"cmd TelloAction no manejado: {cmd!r}")
        resp.rc = 1   # 1 = OK (tello_ros)
        return resp


def main():
    rclpy.init()
    n = TelloActionShim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
