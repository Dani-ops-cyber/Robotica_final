import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QRCodeSubscriber(Node):
    def __init__(self):
        super().__init__('qr_code_subscriber')
        # Suscripción al tópico 'cmd_vel'
        self.subscription = self.create_subscription(
            String,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.subscription  # Evita advertencias de variable no usada
        self.last_message_time = self.get_clock().now()
        self.get_logger().info("Nodo QRCodeSubscriber iniciado. Escuchando mensajes en 'cmd_vel'.")

    def listener_callback(self, msg):
        # Actualizar el tiempo del último mensaje recibido
        self.last_message_time = self.get_clock().now()
        # Mostrar el mensaje recibido
        self.get_logger().info(f"Mensaje recibido del QR: {msg.data}")

    def check_message_status(self):
        # Verificar si se recibió un mensaje en los últimos 2 segundos
        time_since_last_message = (self.get_clock().now() - self.last_message_time).nanoseconds / 1e9
        if time_since_last_message > 2:
            self.get_logger().warn("No se ha recibido ningún mensaje reciente.")

def main(args=None):
    rclpy.init(args=args)
    qr_subscriber = QRCodeSubscriber()

    try:
        # Ciclo principal
        while rclpy.ok():
            rclpy.spin_once(qr_subscriber, timeout_sec=1.0)
            qr_subscriber.check_message_status()
    except KeyboardInterrupt:
        qr_subscriber.get_logger().info("Cerrando el nodo QRCodeSubscriber...")
    finally:
        qr_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

