#!/usr/bin/env python3

# ye toh hogya Shabang ji

# importing ROS 2 Python client library
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# defining a Node that publishes to /s1
class Signal1Node(Node):
    def __init__(self):
        super().__init__('signal1_node')  # name of the node
        self.publisher_ = self.create_publisher(String, '/s1', 10)  # to publish to /s1 topic
        self.timer_period = 1.0  # publishes every 1 second
        
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.start_time = self.get_clock().now()
        # self.state = 'green'
        # above three lines have been commented bcoz isse infinite loop ja raha tha, which we don't want.  
        # We'd rather our program stops.

        self.timer = self.create_timer(1.0, self.publish_signal)
        self.counter = 0

    # previous callback block (wrong)
    # def timer_callback(self):
    #     elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
    #     if elapsed_time > 10.0:
    #         self.state = 'red'
    #     msg = String()
    #     msg.data = self.state
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f'Publishing to /s1: "{msg.data}"')

    def publish_signal(self):
        msg = String()
        if self.counter < 10:
            msg.data = 'green'
        elif self.counter < 20:
            msg.data = 'red'
        else:
            self.get_logger().info('20 seconds done. Shutting down signal 1 node.')
            rclpy.shutdown()
            return

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing to /s1: "{msg.data}"')
        self.counter += 1

# standard ROS 2 Python entry point
def main(args=None):
    rclpy.init(args=args)  # to initialize ROS 2
    node = Signal1Node()   # creates the node
    try:
        rclpy.spin(node)   # to keep node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # for cleanup
        rclpy.shutdown()     # to shutdown ROS 2

# now this runs main() when script is executed
if __name__ == '__main__':
    main()
