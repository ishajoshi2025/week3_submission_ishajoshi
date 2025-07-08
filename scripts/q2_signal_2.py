#!/usr/bin/env python3

# pyaara Shebang

# importing basic ROS 2 Python libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Message type, to send simple text

# defining a new ROS 2 node class
class Signal2Node(Node):
    def __init__(self):
        super().__init__('signal_2')  # initializing the node with name 'signal_2'

        self.publisher_ = self.create_publisher(String, '/s2', 10)  # publisher for /s2

        self.subscription = self.create_subscription(
            String, '/s1', self.listener_callback, 10
        )
        # message type, topic name, callback, queue size

    def listener_callback(self, msg):
        self.get_logger().info(f'Received from /s1: {msg.data}') 
        response_msg = String()  # to publish on /s2

        if msg.data == 'green':
            response_msg.data = 'red'
        elif msg.data == 'red':
            response_msg.data = 'green'
        else:
            self.get_logger().warn(f'Unexpected data on /s1: {msg.data}')
            return

        self.publisher_.publish(response_msg)
        self.get_logger().info(f'Received on /s1: {msg.data} → Publishing on /s2: {response_msg.data}')

# the main function to run the node
def main(args=None):
    rclpy.init(args=args)
    node = Signal2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# Required to execute main()
if __name__ == '__main__':
    main()
