import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.lidar_subscriber = self.create_subscription(LaserScan, '/base_scan', self.lidar_callback, 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_msg = Twist()

    def lidar_callback(self, msg: LaserScan):
        # Check if there's any obstacle closer than 20 cm (0.2 meters)
        min_distance = 1.5  # 20 cm
        front_distances = msg.ranges[len(msg.ranges) // 2 - 15:len(msg.ranges) // 2 + 15]  # Front scan 30 distances
        min_front_distance = min(front_distances) if front_distances else float('inf')

        # Logic to avoid obstacles
        if min_front_distance < min_distance:
            # If there's an obstacle too close, turn right
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 3.141516  # Rotate right
        else:
            # Move forward if no obstacles in front
            self.twist_msg.linear.x = 0.3  # Move forward
            self.twist_msg.angular.z = 0.0  # No rotation

        # Publish the movement command
        self.cmd_publisher.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
