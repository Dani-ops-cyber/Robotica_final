import cv2
import pyzbar.pyzbar as pyzbar
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class QRCodePublisher(Node):
    def __init__(self):
        super().__init__('qr_code_publisher')
        # Crear un publicador para el tópico 'cmd_vel'
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Nodo QRCodePublisher iniciado. Publicando comandos en 'cmd_vel'.")

        # Diccionario de comandos según el código QR
        self.commands = {
            'A1': 'avanzar',
            'B1': 'retroceder',
            'C1': 'girar_90_derecha',     ##giro 90 derecha
            'D1': 'girar_90_izquierda',   ##Giro 90 izquierda 
            'E1': 'girar_180_derecha',    ##Giro 180 derecha
            'F1': 'girar_180_izquierda',  ##Giro 180 izquierda
        }

        # Inicializar la videocaptura
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara en /dev/video0.")
            exit()

        # Configurar la resolución de la cámara
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Ciclo principal para leer y procesar QR
        self.timer = self.create_timer(0.05, self.read_and_publish_qr)  # Intervalo reducido para lecturas rápidas

        # Bandera para controlar si el robot está ocupado
        self.is_busy = False

    def read_and_publish_qr(self):
        # No procesar si el robot está ocupado
        if self.is_busy:
            return

        # Leer frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("No se pudo leer el frame de la cámara.")
            return

        # Convertir el frame a escala de grises para acelerar el procesamiento
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detectar códigos QR en el frame
        for codes in pyzbar.decode(gray_frame):
            # Decodificar información del QR
            info = codes.data.decode('utf-8')

            # Mostrar el nombre del QR detectado
            self.get_logger().info(f"QR detectado: {info}")

            # Ejecutar acción correspondiente
            if info in self.commands:
                action = self.commands[info]
                self.execute_action(action)
            else:
                self.get_logger().warn(f"QR desconocido: {info}")

            # Procesar solo el primer QR detectado en este frame
            break

    def move_forward(self, distance, speed):
        duration = distance / speed
        self.publish_twist(speed, 0.0)
        self.get_logger().info(f"Moviendo hacia adelante: Velocidad={speed}, Duración={duration}s")
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def move_backward(self, distance, speed):
        duration = distance / speed
        self.publish_twist(-speed, 0.0)
        self.get_logger().info(f"Retrocediendo: Velocidad={speed}, Duración={duration}s")
        time.sleep(duration)
        self.publish_twist(0.0, 0.0)

    def execute_action(self, action):
        # Marcar al robot como ocupado
        self.is_busy = True

        linear_speed = 0.6 * 0.6  # Velocidad reducida al 50%
        angular_speed = 1.0 * 0.5  # Velocidad angular reducida al 50%

        # Detener la captura de QR temporalmente
        self.cap.release()

        if action == 'avanzar':
            self.move_forward(2.0, linear_speed)  # Avanza 2 metros
        elif action == 'retroceder':
            self.move_backward(2.0, linear_speed)  # Retrocede 2 metros
        elif action == 'girar_90_derecha':
            duration = 0.375 / angular_speed  # Tiempo para girar 45° con velocidad reducida
            self.publish_twist(0.0, -angular_speed)
            self.get_logger().info(f"Acción: Girar 90° a la derecha, Duración: {duration}s")
            time.sleep(duration)
            self.publish_twist(0.0, 0.0)
            
            ##y au avance
            self.move_forward(2.0, linear_speed)
            ##
        elif action == 'girar_90_izquierda':
            duration = 0.83 / angular_speed  # Tiempo para girar 45° con velocidad reducida
            self.publish_twist(0.0, angular_speed)
            self.get_logger().info(f"Acción: Girar 90° a la izquierda, Duración: {duration}s")
            time.sleep(duration)
            self.publish_twist(0.0, 0.0)
            
            ##y au avance
            self.move_forward(2.0, linear_speed) 
            
        elif action == 'girar_180_derecha':
            duration = 0.8 / angular_speed  # Tiempo para girar 180° con velocidad reducida
            self.publish_twist(0.0, -angular_speed)
            self.get_logger().info(f"Acción: Girar 180° a la derecha, Duración: {duration}s")
            time.sleep(duration)
            self.publish_twist(0.0, 0.0)
        elif action == 'girar_180_izquierda':
            duration = 1.8 / angular_speed  # Tiempo para girar 180° con velocidad reducida
            self.publish_twist(0.0, angular_speed)
            self.get_logger().info(f"Acción: Girar 180° a la izquierda, Duración: {duration}s")
            time.sleep(duration)
            self.publish_twist(0.0, 0.0)

        # Marcar al robot como listo para procesar otro QR
        self.is_busy = False

        # Reiniciar la captura de QR
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara al reiniciar.")

    def publish_twist(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.publisher.publish(twist_msg)

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

