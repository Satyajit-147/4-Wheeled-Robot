import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleStopper(Node):
    def __init__(self):
        super().__init__('obstacle_stopper')

        # Publisher: sends velocity commands to robot
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: listens to LaserScan for obstacle detection
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Subscribing to /cmd_vel 
        self.cmd_vel = Twist()
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg  # Store latest command to forward if safe

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        self.get_logger().info(f"Closest object: {min_distance:.2f} m")

        if min_distance < 0.5:
            self.get_logger().warn("Obstacle too close! Stopping robot.")
            self.publisher.publish(Twist())  # Stop
        else:
            self.publisher.publish(self.cmd_vel)  # Forward latest cmd_vel if safe

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

