import sys
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Para recibir comandos como texto
from geometry_msgs.msg import Twist

# Añadir el directorio actual al PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot import Robot  # Ahora debería encontrar robot.py correctamente

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Suscripción al tópico de comandos del usuario
        self.subscription = self.create_subscription(
            String,
            'user_cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publicador para visualizar los movimientos generados
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Inicialización del robot
        self.robot = Robot()

    def listener_callback(self, msg):
        # Interpretar el comando recibido
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received command: {command}')
        
        # Configuración de velocidades
        linear_x = 0.0
        angular_z = 0.0

        if command == 'w':  # Adelante
            linear_x = 0.5
        elif command == 's':  # Atrás
            linear_x = -0.5
        elif command == 'a':  # Izquierda
            angular_z = 0.5
        elif command == 'd':  # Derecha
            angular_z = -0.5
        elif command == 'p':  # Parar
            linear_x = 0.0
            angular_z = 0.0
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        # Asumimos que el robot usa dos motores
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        # Configuración de los motores
        self.robot.set_motors(left_speed, right_speed)

        # Publicar el movimiento en cmd_vel para visualización
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

