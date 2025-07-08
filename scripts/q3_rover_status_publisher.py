#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
from kratos_isha.msg import RoverStatus  # Import your custom message

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('rover_status_publisher')

        # Create a publisher for your custom RoverStatus message
        self.publisher_ = self.create_publisher(RoverStatus, '/rover/status', 10)

        # Publish every 1 second
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Simulated data values (in a real robot, this would come from sensors or calculations)
        self.distance = 0.0
        self.time_elapsed = 0.0

    def timer_callback(self):
        # Create and populate a RoverStatus message
        msg = RoverStatus()

        # Fill Twist (velocity)
        msg.velocity.linear.x = 1.5
        msg.velocity.angular.z = 0.3

        # Distance travelled (simulated)
        self.distance += 1.5
        msg.distance = self.distance

        # Pose (coordinates)
        msg.coordinates.position.x = self.distance
        msg.coordinates.position.y = 2.0
        msg.coordinates.position.z = 0.0

        # Battery power (simulated)
        msg.battery = 75.0

        # Time of travel (in seconds)
        self.time_elapsed += 1.0
        msg.time = self.time_elapsed

        # Publish
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing RoverStatus: [distance={msg.distance}, battery={msg.battery}, time={msg.time}]'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RoverStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


