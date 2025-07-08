# kratos_isha

## Overview

**Week 3 Assignment**

 *Nodes that demonstrate **publishing** and **subscribing** to messages using Python (`rclpy`). Here, the idea is to understand how two different programs communicate using ROS 2 topics.


## Question 1: Publish and Subscribe

**Task**
> To write a node that **publishes** the string `"Hello World!"` on a topic `/new` at a rate of **15 messages per second**
> To write a node that **subscribes** to the same topic `/new` and **prints** the received message to the terminal


## Files

 `scripts/q1_publisher.py` Publishes the string `"Hello World!"` to the topic `/new` 15 times every second.
 `scripts/q1_subscriber.py` Subscribes to `/new` and prints any received messages to the terminal.


##What We Did

## Publisher Node:
- Created a node using `rclpy`.
- It publishes `"Hello World!"` on a topic named `/new`.
- The publishing happens every **1/15th of a second** (i.e., 15 times per second).

It is like a radio station *announcing a message repeatedly*.

## Subscriber Node:
- Another node, listens (*subscribes*) to the same topic `/new`.
- Every time a message is published, it *prints* it to the terminal.

This is like a radio receiver, it listens to the broadcast and displays it*.


## Code Explanations

`#!/usr/bin/env python3` :  *shebang*, makes our script executable

`import rclpy` : *ROS 2 Python library*, gives access to all ROS 2 functions

`from rclpy.node import Node` : creates a custom node by *extending the base `Node` class*

`from std_msgs.msg import String` : *standard message type (`String`)* to send plain text messages

Publisher: `create_publisher(String, 'new', 10)` : Creates a publisher that will send messages of type `String` on the topic `/new`
`10` is the *queue size*, helps buffer messages if *the subscriber is slow*

Timer: `self.create_timer(1/15, self.publish_message)` : tells ROS to *run `self.publish_message()` every 1/15 seconds*, i.e., at 15 Hz.

Subscriber: `self.create_subscription(String, 'new', self.listener_callback, 10)` : listens to `/new`, and when a msg is received, it calls `listener_callback()` with the msg

##"Build Instructions" (chatgpt kare the ye tbh)

Make sure you are in your ROS 2 workspace:

cd ~/ros2_ws
colcon build
source install/setup.bash








## Question 2: Signal Coordination System


**Task**

> To create two ROS 2 nodes that mimic the behavior of two traffic signals (S1 and S2).

Signal 1 (S1) must publish "green" on topic /s1 for 10 seconds, then "red" for 10 seconds, and then stop.

Signal 2 (S2) should listen to /s1 and publish the opposite color on /s2.

## Files

'scripts/q2_signal_1.py' Publishes "green" for 10 seconds, then "red" for 10 seconds, to topic /s1.

'scripts/q2_signal_2.py' Subscribes to /s1, and publishes "red" on /s2 while /s1 is "green", and "green" on /s2 when /s1 is "red".


##What We Did



##Signal 1 Node:


- Created a publisher node that alternates signals:
- For the first 10 seconds, it publishes "green" every second.
- For the next 10 seconds, it publishes "red" every second.
- After 20 seconds, it shuts itself down (to avoid infinite publishing).
- A counter keeps track of how many seconds have passed.

- Timer callback publish_signal() decides what to send based on the counter.
- This is like a timer-based signal controller that just flips after 10 seconds.


##Signal 2 Node:


- A subscriber node that listens to /s1.
- It publishes the opposite signal on /s2:
- If it hears "green" on /s1, it publishes "red" on /s2.
- If it hears "red" on /s1, it publishes "green" on /s2.
- This is like a *smart signal* that listens to S1 and adjusts accordingly.



## Code Explanations


'#!/usr/bin/env python3' : Shebang line – allows the script to run as an executable.

'rclpy and Node' : ROS 2 base classes to define nodes and manage communication.

'String' : Standard message type used to send simple text (like "green" and "red").


**'q2_signal_1.py' Specifics:**

'self.create_timer(1.0, self.publish_signal)' : Tells the node to call publish_signal() every second.
'self.counter' : Tracks the number of seconds passed.

if self.counter < 10:
Send "green" for first 10 seconds.

elseif self.counter < 20:
Then send "red" for the next 10 seconds.

'rclpy.shutdown()' : Stops the node after 20 seconds.


**'q2_signal_2.py' Specifics:**

'self.create_subscription(String, '/s1', self.listener_callback, 10)' :  Subscribes to topic /s1, and runs listener_callback() every time a new message comes.

if msg.data == 'green'
then response_msg.data = 'red'

*Opposite logic: if signal 1 is "green", signal 2 becomes "red"*

'self.publisher_.publish(response_msg)' : Sends the determined message to /s2.

'get_logger().info(...)' : Helps in printing the interactions in the terminal for debugging.










## Question 3: Custom Message Publishing from Mars Rover


**Task**
> A Mars rover must broadcast multiple pieces of status information through a single custom message on a topic.
> The message should include:
-Velocity (linear + angular) → geometry_msgs/Twist
-Distance travelled → float64
-Coordinates → geometry_msgs/Pose
-Battery level → float32
-Time of travel → builtin_interfaces/Duration



## Files

 
'msg/RoverStatus.msg' : Defines a custom message format that combines all required fields: velocity, position, distance, battery, and time.
'scripts/q3_rover_status_publisher.py' : A Python ROS 2 node that simulates a Mars rover and publishes its status every 1 second using the custom message.

##What We Did

-We designed a custom message, so that all necessary rover data is packed into one structured message.

-Created a .msg file with fields of existing types (like Twist, Pose) along with basic types like float64 and float32.

-We then created a publisher node that simulates:
Constant velocity
Incrementing distance and time
Static coordinates (with x = distance, y = 2.0)
A fixed battery level (e.g., 75%)

- This node publishes a new status message every second on topic /rover/status.

"Imagine it as a black box telemetry module on the rover — every second, it reports back its "health and location"."



## Understanding the Custom Message

Inside msg/RoverStatus.msg, we define the following:

geometry_msgs/Twist velocity
float64 distance_traveled
geometry_msgs/Pose position
float32 battery_level
builtin_interfaces/Duration time_traveled

> Each of these fields allows us to re-use existing ROS message types (like Twist, Pose, Duration) to represent complex data like vectors, positions, and timestamps — while also adding simpler float fields like distance and battery.



## Code Explanations


'q3_rover_status_publisher.py' : imports:

from 'kratos_isha.msg import RoverStatus' :
imports the custom message we defined in msg/RoverStatus.msg.


> Publisher creation:
'self.publisher_ = self.create_publisher(RoverStatus, '/rover/status', 10)' : Creates a publisher that sends RoverStatus messages to the topic /rover/status with a queue size of 10.

> Timer:
'self.timer = self.create_timer(1.0, self.timer_callback)' : Calls the timer_callback() function every 1 second — this is where we publish the message.


> Simulated data:

self.distance = 0.0
self.time_elapsed = 0.0

-We initialize dummy values to simulate the rover's behavior over time.

'
In 'timer_callback()' -
msg.velocity.linear.x = 1.5
msg.velocity.angular.z = 0.3

-We simulate constant forward linear velocity and a slight angular rotation.


self.distance += 1.5
msg.distance_traveled = self.distance

-Pretend the rover travels 1.5 meters every second.


msg.position.position.x = self.distance
msg.position.position.y = 2.0

-We hard-code the position's y coordinate, and use distance for x to simulate forward motion.

'
msg.battery_level = 75.0
-We assume the battery is at 75% for simplicity.


self.time_elapsed += 1.0
msg.time_traveled.sec = int(self.time_elapsed)
-We track how long the rover has been "traveling" and convert it to Duration.


Final publishing step:
self.publisher_.publish(msg)
-Sends the message out on /rover/status.

self.get_logger().info(f'Publishing RoverStatus: [distance={msg.distance_traveled}, battery={msg.battery_level}, time={msg.time_traveled}]')
-Prints the info to terminal so we know it's working.



'
'from geometry_msgs.msg import Twist, Pose' -	Reusing existing ROS 2 messages for velocity and coordinates
'from kratos_isha.msg import RoverStatus' - Importing our own custom message type
'self.create_timer(1.0, self.timer_callback)' -	Schedule timer_callback() to run every second
'msg.velocity.linear.x = 1.5' -	Set constant linear velocity in X direction
'self.distance += 1.5'	- Simulating distance as speed × time
'msg.coordinates.position.x = self.distance' -	Updating position to reflect distance traveled
'rclpy.shutdown()' - Properly shuts down the node when done


#Field          	#Dummy Value    	# Why?
velocity.linear.x	1.5 m/s	             Simulates forward motion
velocity.angular.z	0.3 rad/s          	 Simulates light turning
distance	        0.0 → +1.5/s	     Matches linear velocity
battery	            75.0%	             Fixed to simulate moderate charge
time_elapsed	    0.0 → +1/s	         Tracks how long the rover’s been moving
coordinates.x	    =  distance	         Keeps spatial logic simple






##Question 4: ROS 2 Clock System


**Task**

To implement a clock system using ROS 2.

The system publishes time as HH:MM:SS on a topic /clock, using three counters:
/second (increments every second)
/minute (increments after 60 seconds)
/hour (increments after 60 minutes)
The full time (HH:MM:SS format) is published as a String on /clock.



##Files

scripts/q4_clock_publisher.py : Implements a publisher node that simulates a live ticking clock by publishing: Integer counts on /second, /minute, and /hour
A formatted clock string on /clock



##What We Did

We created a single publisher node that simulates a clock:
The node starts counting seconds from 0.
Every 60 seconds, it resets seconds to 0 and increments the minute counter.
Every 60 minutes, it resets minutes to 0 and increments the hour counter.

After each tick, it publishes the current time in HH:MM:SS format on the /clock topic using String.

It also publishes the raw values of /second, /minute, and /hour as Int32 messages.

This mimics a real digital clock with independent second, minute, and hour hands — except in software.



##Code Explanation

#!/usr/bin/env python3 : Shebang line — allows the script to be executed like a program

from std_msgs.msg import Int32, String :
Int32 is used for /second, /minute, and /hour counters
String is used for formatted time string on /clock




Node Setup:

super().__init__('clock_publisher') : initializes a ROS 2 node with the name clock_publisher



Publishers:

self.second_pub = self.create_publisher(Int32, '/second', 10)
self.minute_pub = self.create_publisher(Int32, '/minute', 10)
self.hour_pub = self.create_publisher(Int32, '/hour', 10)
self.clock_pub = self.create_publisher(String, '/clock', 10)

Four publishers are created to send time data on their respective topics.



Timer:

self.timer = self.create_timer(1.0, self.timer_callback) : this sets a timer that calls timer_callback() every 1 second, simulating real-time ticking

Time Logic:

self.second += 1
if self.second >= 60:
    self.second = 0
    self.minute += 1
Every second, we increment the second counter.

If seconds reach 60, reset it to 0 and increment the minute counter.

Similarly, when minutes reach 60, increment hour.



Clock Formatting:

clock_msg.data = f'{self.hour:02d}:{self.minute:02d}:{self.second:02d}' : this creates a string like 04:05:09 with zero-padding



Final Publishing:

self.second_pub.publish(second_msg)
self.minute_pub.publish(minute_msg)
self.hour_pub.publish(hour_msg)
self.clock_pub.publish(clock_msg)
All four messages are published at once, every second.



