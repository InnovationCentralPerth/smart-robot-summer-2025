
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class NetworkTalker(Node):
    def __init__(self):
        super().__init__('network_talker')
        self.publisher_ = self.create_publisher(String, 'network_test_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        try:
            hostname = socket.gethostname()
            ip_address = socket.gethostbyname(hostname)
        except:
            ip_address = "Unknown IP"
        self.get_logger().info(f'Network Talker started on {ip_address}. Publishing to "network_test_topic"...')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Raspberry Pi! Message count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    network_talker = NetworkTalker()
    try:
        rclpy.spin(network_talker)
    except KeyboardInterrupt:
        pass
    network_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
