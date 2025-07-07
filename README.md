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
