import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math
import random

class DriftSimulator(Node):
    def __init__(self):
        super().__init__('drift_simulator')
        
        # Parámetro para la magnitud del drift
        self.declare_parameter('drift_magnitude', 0.1)  # 10 cm/s por defecto
        self.drift_magnitude = self.get_parameter('drift_magnitude').get_parameter_value().double_value
        
        # Generar una dirección aleatoria (ángulo entre 0 y 2*pi)
        theta = random.uniform(0, 2 * math.pi)
        
        # Calcular los componentes X e Y del drift constante
        self.drift_x = self.drift_magnitude * math.cos(theta)
        self.drift_y = self.drift_magnitude * math.sin(theta)
        
        self.get_logger().info(f'Iniciando simulador de drift. Magnitud: {self.drift_magnitude:.3f} m/s')
        self.get_logger().info(f'Dirección elegida aleatoriamente: X={self.drift_x:.3f}, Y={self.drift_y:.3f}')
        
        # Suscriptor a los comandos limpios del controlador
        self.sub = self.create_subscription(
            Twist,
            '/drone1/cmd_vel_clean',
            self.cmd_callback,
            10
        )
        
        # Publicador de comandos contaminados con drift al simulador
        self.pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        
        # Publicador de información de drift para el plotter
        self.drift_info_pub = self.create_publisher(Point, '/drone1/drift_info', 10)
        self.drift_timer = self.create_timer(1.0, self.publish_drift_info)
        
    def publish_drift_info(self):
        msg = Point()
        msg.x = self.drift_x
        msg.y = self.drift_y
        msg.z = 0.0
        self.drift_info_pub.publish(msg)
        
    def cmd_callback(self, msg):
        drifted_msg = Twist()
        
        # Sumar el drift constante a las velocidades X e Y
        drifted_msg.linear.x = msg.linear.x + self.drift_x
        drifted_msg.linear.y = msg.linear.y + self.drift_y
        
        # Mantener Z y los ángulos sin drift extra (o se podría añadir si se desea)
        drifted_msg.linear.z = msg.linear.z
        drifted_msg.angular.x = msg.angular.x
        drifted_msg.angular.y = msg.angular.y
        drifted_msg.angular.z = msg.angular.z
        
        self.pub.publish(drifted_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DriftSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
