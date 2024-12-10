import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')

        # Crear el publicador para el tópico cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Instrucciones para el usuario
        self.get_logger().info("Control del robot con las siguientes teclas:")
        self.get_logger().info("'w': Adelante, 's': Atrás, 'a': Izquierda, 'd': Derecha, 'p': Detener")
        self.get_logger().info("Presiona 'q' para salir.")

    def send_command(self, command):
        """
        Convierte un comando en velocidades lineales y angulares y lo publica.
        """
        msg = Twist()

        if command == 'w':
            msg.linear.x = 0.5  # Adelante
            msg.angular.z = 0.0
            self.get_logger().info("Comando enviado: Adelante")
        elif command == 's':
            msg.linear.x = -0.5  # Atrás
            msg.angular.z = 0.0
            self.get_logger().info("Comando enviado: Atrás")
        elif command == 'a':
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Izquierda
            self.get_logger().info("Comando enviado: Izquierda")
        elif command == 'd':
            msg.linear.x = 0.0
            msg.angular.z = -0.5  # Derecha
            self.get_logger().info("Comando enviado: Derecha")
        elif command == 'p':
            msg.linear.x = 0.0
            msg.angular.z = 0.0  # Detener
            self.get_logger().info("Comando enviado: Detener")
        else:
            self.get_logger().warn("Comando inválido. Usa 'w', 's', 'a', 'd', 'p' o 'q'.")
            return

        # Publicar el mensaje
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()

    try:
        while rclpy.ok():
            # Leer comando del usuario
            command = input("Ingresa un comando ('w', 's', 'a', 'd', 'p', 'q' para salir): ").strip().lower()

            if command == 'q':
                print("Saliendo del programa...")
                break

            # Enviar el comando ingresado
            node.send_command(command)

    except KeyboardInterrupt:
        print("\nPrograma interrumpido.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

