import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_msg = Twist()
        self.remaining_rotation = None  # Radianes restantes para rotar
        self.timer = self.create_timer(0.1, self.rotate_twice)  # Ejecutar cada 0.1 segundos

    def rotate_twice(self):
        if self.remaining_rotation is None:
            # Inicializa la rotación: 2 vueltas = 4π radianes
            self.remaining_rotation = 4 * math.pi
        
        # Define la velocidad angular (ajusta según necesites)
        angular_speed = 1.0  # rad/s
        
        if self.remaining_rotation > 0:
            # Continuar girando
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = angular_speed
            self.remaining_rotation -= angular_speed * 0.1  # Reducir según el tiempo del timer
        else:
            # Detener el giro cuando termine
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.remaining_rotation = None
            self.get_logger().info('Rotación completada')
            self.timer.cancel()  # Detener el temporizador
        
        # Publicar el comando
        self.cmd_publisher.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

