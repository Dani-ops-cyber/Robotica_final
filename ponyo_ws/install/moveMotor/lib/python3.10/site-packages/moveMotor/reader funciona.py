import cv2
import pyzbar.pyzbar as pyzbar
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class QRCodePublisher(Node):
    def __init__(self):
        super().__init__('qr_code_publisher')
        # Crear un publicador para el tópico 'cmd_vel'
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Nodo QRCodePublisher iniciado. Publicando comandos en 'cmd_vel'.")

        # Diccionario de comandos según el código QR
        self.commands = {
            'A1': (0.5, 0.0),  # Avanzar: linear_x = 0.5, angular_z = 0.0
            'B1': (0.0, -1.0),  # Girar a la derecha: linear_x = 0.0, angular_z = -1.0
            'C1': (0.0, 1.0)  # Girar a la izquierda: linear_x = 0.0, angular_z = 1.0
        }

        # Inicializar la videocaptura
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara en /dev/video0.")
            exit()

        # Ciclo principal
        self.timer = self.create_timer(0.1, self.read_and_publish_qr)

        # Estado del último mensaje detectado
        self.last_message_time = self.get_clock().now()

    def read_and_publish_qr(self):
        # Leer frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo leer el frame de la cámara.")
            return

        detected = False

        # Detectar códigos QR en el frame
        for codes in pyzbar.decode(frame):
            # Decodificar información del QR
            info = codes.data.decode('utf-8')
            detected = True

            # Obtener el comando asociado al QR
            if info in self.commands:
                linear_x, angular_z = self.commands[info]
                # Reducir la velocidad al 50%
                linear_x *= 0.5
                angular_z *= 0.5
                self.publish_twist(linear_x, angular_z)
                self.get_logger().info(f"Publicado: QR={info}, Linear_x={linear_x}, Angular_z={angular_z}")
            else:
                self.get_logger().warn(f"QR desconocido: {info}")

        # Si no se detectó un QR recientemente, detener el vehículo
        if not detected:
            time_since_last_message = (self.get_clock().now() - self.last_message_time).nanoseconds / 1e9
            if time_since_last_message > 0.5:  # Detener si han pasado más de 0.5s sin mensaje
                self.publish_twist(0.0, 0.0)
                self.get_logger().info("No se detectó un QR. Vehículo detenido.")

    def publish_twist(self, linear_x, angular_z):
        # Publicar mensaje Twist
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.publisher.publish(twist_msg)
        self.last_message_time = self.get_clock().now()

    def stop(self):
        self.get_logger().info("Cerrando el nodo QRCodePublisher...")
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = QRCodePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

