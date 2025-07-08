#!/usr/bin/env python3

# our very own Shebang that ensures this script runs with Python 3 interpreter when executed directly

import rclpy  # core ROS 2 Python client library
from rclpy.node import Node  # used to define and create custom ROS 2 nodes
from std_msgs.msg import Int32, String  # message types: *Int32 for time counts*, *String for final clock output*

# defining a custom Node class that acts like a digital clock
class ClockPublisher(Node):
    def __init__(self):
        # initializing the parent Node class with a name
        super().__init__('clock_publisher')

        # creating 4 publishers:
        # - One for each unit of time (second, minute, hour)
        # - One for combined formatted string /clock
        self.second_pub = self.create_publisher(Int32, '/second', 10)
        self.minute_pub = self.create_publisher(Int32, '/minute', 10)
        self.hour_pub = self.create_publisher(Int32, '/hour', 10)
        self.clock_pub = self.create_publisher(String, '/clock', 10)

        # internal counters to simulate the clock
        self.seconds = 0
        self.minutes = 0
        self.hours = 0

        # timer: calls update_time() once every second (1.0 seconds = 1Hz)
        self.timer = self.create_timer(1.0, self.update_time)

    def update_time(self):
        # increase seconds by 1 every time this function is called
        self.seconds += 1

        # if seconds reach 60, reset and increment minute
        if self.seconds >= 60:
            self.seconds = 0
            self.minutes += 1

        # if minutes reach 60, reset and increment hour
        if self.minutes >= 60:
            self.minutes = 0
            self.hours += 1

        # to create messages to publish each unit
        sec_msg = Int32()  # Message for /second
        min_msg = Int32()  # Message for /minute
        hr_msg = Int32()   # Message for /hour

        # to assign current counter values to each message
        sec_msg.data = self.seconds
        min_msg.data = self.minutes
        hr_msg.data = self.hours

        # to create the final time string (format: HH:MM:SS)
        clock_msg = String()
        clock_msg.data = f'{self.hours:02}:{self.minutes:02}:{self.seconds:02}'

        # publishing the messages on their respective topics
        self.second_pub.publish(sec_msg)
        self.minute_pub.publish(min_msg)
        self.hour_pub.publish(hr_msg)
        self.clock_pub.publish(clock_msg)

        # we'd also log the time in the terminal for debugging/monitoring
        self.get_logger().info(f'Time → {clock_msg.data}')

# initializes the node and keeps it running
def main(args=None):
    rclpy.init(args=args)  # initializing ROS 2
    node = ClockPublisher()  # to instantiate the custom node

    try:
        rclpy.spin(node)  # to keep the node alive and responsive
    except KeyboardInterrupt:
        # Graceful exit if user presses Ctrl+c (force stop)
        pass
    finally:
        # Clean up: destroy node and shut down ROS 2
        node.destroy_node()
        rclpy.shutdown()

# ensures main() runs only when this file is executed directly
if __name__ == '__main__':
    main()
