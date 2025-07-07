#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
 #same cheezein hain ye toh
#Python, ROS 2 and message type have been set up



class HelloSubscriber(Node):
    def __init__(self):
        super().__init__('hello_subscriber')
        self.subscription = self.create_subscription(String,'/new',self.listener_callback,10) 
                                  # Message type, Topic to listen to,Function to call when message received, Queue size
        self.subscription  # Prevent unused variable warning 



    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')



def main(args=None):
    rclpy.init(args=args)
    node = HelloSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()




