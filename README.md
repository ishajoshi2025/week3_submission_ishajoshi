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
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash








## Question 2: Signal Coordination System


**Task**

To create two ROS 2 nodes that mimic the behavior of two traffic signals (S1 and S2).

Signal 1 (S1) must publish "green" on topic /s1 for 10 seconds, then "red" for 10 seconds, and then stop.

Signal 2 (S2) should listen to /s1 and publish the opposite color on /s2.

Files
scripts/q2_signal_1.py
Publishes "green" for 10 seconds, then "red" for 10 seconds, to topic /s1.

scripts/q2_signal_2.py
Subscribes to /s1, and publishes "red" on /s2 while /s1 is "green", and "green" on /s2 when /s1 is "red".

What We Did
Signal 1 Node:
Created a publisher node that alternates signals:

For the first 10 seconds, it publishes "green" every second.

For the next 10 seconds, it publishes "red" every second.

After 20 seconds, it shuts itself down (to avoid infinite publishing).

A counter keeps track of how many seconds have passed.

Timer callback publish_signal() decides what to send based on the counter.

This is like a timer-based signal controller that just flips after 10 seconds.

Signal 2 Node:
A subscriber node that listens to /s1.

It publishes the opposite signal on /s2:

If it hears "green" on /s1, it publishes "red" on /s2.

If it hears "red" on /s1, it publishes "green" on /s2.

This is like a smart signal that listens to S1 and adjusts accordingly.

Code Explanations
#!/usr/bin/env python3 : Shebang line – allows the script to run as an executable.

rclpy and Node : ROS 2 base classes to define nodes and manage communication.

String : Standard message type used to send simple text (like "green" and "red").

q2_signal_1.py Specifics:
self.create_timer(1.0, self.publish_signal)
Tells the node to call publish_signal() every second.

self.counter
Tracks the number of seconds passed.

if self.counter < 10:
Send "green" for first 10 seconds.

elif self.counter < 20:
Then send "red" for the next 10 seconds.

rclpy.shutdown()
Stops the node after 20 seconds.

q2_signal_2.py Specifics:
self.create_subscription(String, '/s1', self.listener_callback, 10)
Subscribes to topic /s1, and runs listener_callback() every time a new message comes.

if msg.data == 'green': response_msg.data = 'red'
Opposite logic: if signal 1 is "green", signal 2 becomes "red".

self.publisher_.publish(response_msg)
Sends the determined message to /s2.

get_logger().info(...)
Helps in printing the interactions in the terminal for debugging.

