#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import collections

class OptitrackSimulator(Node):
    def __init__(self):
        super().__init__('optitrack_simulator')
        
        # Suscriptor a la odometría de Gazebo (verdadera posición en simulación)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone1/odom',
            self.odom_callback,
            10
        )
        
        # Publicador que simula el tópico de OptiTrack
        self.opti_pub = self.create_publisher(
            PoseStamped,
            '/optitrack/rigid_body',
            10
        )
        self.drone_pose_pub = self.create_publisher(
            PoseStamped,
            '/drone_pose',
            10
        )
        
        # Latencia configurada en segundos
        self.declare_parameter('latency_sec', 0.1)
        self.latency_sec = self.get_parameter('latency_sec').get_parameter_value().double_value
        
        # Buffer circular para guardar mensajes con su tiempo de recepción
        # Cada elemento será una tupla: (tiempo_recepcion_segundos, mensaje_Odometry)
        self.msg_buffer = collections.deque()
        
        # Temporizador para publicar los mensajes retrasados (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_delayed_pose)

    def odom_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.msg_buffer.append((current_time, msg))

    def publish_delayed_pose(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Revisar el buffer desde el más antiguo (izquierda)
        msg_to_publish = None
        
        while self.msg_buffer:
            # Si el mensaje más antiguo ya cumplió la latencia
            if current_time - self.msg_buffer[0][0] >= self.latency_sec:
                msg_to_publish = self.msg_buffer.popleft()[1]
            else:
                # Si el mensaje más antiguo aún no cumple la latencia, los siguientes tampoco
                break
                
        if msg_to_publish is not None:
            # Convertir de Odometry a PoseStamped simulando OptiTrack
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "optitrack"
            
            # Mantenemos convención Z-up estándar de ROS, al igual que el sistema real
            pose_stamped.pose.position.x = msg_to_publish.pose.pose.position.x
            pose_stamped.pose.position.y = msg_to_publish.pose.pose.position.y
            pose_stamped.pose.position.z = msg_to_publish.pose.pose.position.z
            
            # Orientación (copiamos de odometría real de la simulación)
            pose_stamped.pose.orientation = msg_to_publish.pose.pose.orientation
            
            self.opti_pub.publish(pose_stamped)
            self.drone_pose_pub.publish(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = OptitrackSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
