#!/usr/bin/env python3
import rclpy #importing core ROS 2 PYthon client library, access to all ROS 2 functions
from rclpy.node import Node #to create custom nodes
from std_msgs.msg import String # to publish text messages

class HelloPublisher(Node): #class created
    def __init__(self): #constructor
        super().__init__('hello_publisher') #Node registered as hello_publisher
        self.publisher_ = self.create_publisher(String, '/new', 10)
        timer_period = 1.0 / 15.0  # seconds (15 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = String()

    def timer_callback(self): #runs every 1/15 seconds, implies 15Hz
        self.msg.data = "Hello World !"
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Published: {self.msg.data}')
#syntax hai, stay calm, aadat ho jaayegi : note to self


def main(args=None):
    rclpy.init(args=args) #initializes ROS 2
    node = HelloPublisher() #running node created
    rclpy.spin(node) #"keeps spinning the event wheel"
    node.destroy_node() #cleans up the node and frees memory, once done.
    rclpy.shutdown() #closes the ROS 2 environmant



if __name__ == '__main__':
    main()

