#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from ament_index_python.packages import get_package_share_directory


def _load_camera_info(yaml_path: str) -> CameraInfo:
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    msg = CameraInfo()
    msg.width = data['image_width']
    msg.height = data['image_height']
    msg.distortion_model = data['distortion_model']
    msg.d = data['distortion_coefficients']['data']
    msg.k = data['camera_matrix']['data']
    msg.r = data['rectification_matrix']['data']
    msg.p = data['projection_matrix']['data']
    return msg


class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher_down')

        self.declare_parameter(
            'camera_info_yaml',
            get_package_share_directory('tello_driver') + '/cfg/camera_info_down.yaml'
        )
        self.declare_parameter('image_topic', 'drone1/camera_down')
        self.declare_parameter('camera_info_topic', 'drone1/camera_down/camera_info')

        yaml_path = self.get_parameter('camera_info_yaml').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        try:
            self._camera_info = _load_camera_info(yaml_path)
            self.get_logger().info(f'Camera info cargada desde: {yaml_path}')
        except Exception as e:
            self.get_logger().error(f'No se pudo cargar {yaml_path}: {e}')
            raise

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._info_pub = self.create_publisher(CameraInfo, info_topic, sensor_qos)
        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, sensor_qos
        )
        self.get_logger().info(
            f'Publicando camera_info en "{info_topic}" sincronizado con "{image_topic}"'
        )

    def _image_callback(self, img_msg: Image):
        info = self._camera_info
        info.header.stamp = img_msg.header.stamp
        info.header.frame_id = img_msg.header.frame_id
        self._info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
