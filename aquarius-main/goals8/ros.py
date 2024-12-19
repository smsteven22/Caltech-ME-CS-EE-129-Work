#!/usr/bin/env python3
#
# ros.py
#
# Set up a very simple ROS node to listen for goal/explore commands
# and publish the current pose (position and heading).
#
# Node: /aquarius (this will use your Pi's name)
#
# Publish: ~/pose geometry_msgs/Pose
# Subscribe: ~/goal geometry_msgs/Point
# Subscribe: ~/explore std_msgs/Empty
#
import ctypes
import os
import rclpy
import socket
import time
import threading
from math import pi, sin, cos
from rclpy.node import Node
from rclpy.time import Time, Duration
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Empty
#
# Simple ROS Node Class
#
class ROSNode(Node):
# Initialization.
    def __init__(self, name, shared):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Save the shared data object.
        self.shared = shared

        # Create the publisher for the pose information.
        self.pub = self.create_publisher(Pose, '~/pose', 10)

        # Then create subscribers for goal and explore commands.
        self.create_subscription(Point, '~/goal', self.cb_goal, 10)
        self.create_subscription(Empty, '~/explore', self.cb_explore, 10)

        # Finally create a timer to drive the node.
        self.timer = self.create_timer(1.0, self.cb_timer)

        # Report and return.
        self.get_logger().info("ROS Node '%s' running" % (name))
    
    # Shutdown.
    def shutdown(self):
        # Destroy the timer and shut down the node.
        self.destroy_timer(self.timer)
        self.destroy_node()
    
    # Timer callback. Send the robot's current pose.
    def cb_timer(self):
        # Grab the current x/y and heading from the shared data.
        if self.shared.acquire():
            posx = self.shared.robot_x
            posy = self.shared.robot_y
            head = self.shared.robot_heading
            self.shared.release()

        # Convert the heading into an angle (in radians).
        theta = pi/4 * float(head+2)

        # Populate the ROS message with the data and send. The
        # orientation is encoded as a quaternion.
        msg = Pose()
        msg.position.x = float(posx)
        msg.position.y = float(posy)
        msg.orientation.z = sin(theta/2)
        msg.orientation.w = cos(theta/2)
        self.pub.publish(msg)

    # Goal command callback.
    def cb_goal(self, msg):
        # Extract the goal coordinates from the message.
        xgoal = msg.x
        ygoal = msg.y

        # Report.
        self.get_logger().info("Received goal command (%d,%d)" % (xgoal,ygoal))
        # Inject the goal command into your robot thread, just as if
        # you had typed the goal command in the UI thread.

        if self.shared.acquire():
            self.shared.command = "goal"
            self.shared.command_changed = True

            self.shared.x_desired = xgoal
            self.shared.y_desired = ygoal

            # Release the shared data
            self.shared.release()

    # Explore command callback.
    def cb_explore(self, msg):
        # Report.
        self.get_logger().info("Received explore command")

        # Inject the explore command into your robot thread, just as
        # if you had typed the explore command in the UI thread.

        if self.shared.acquire():
            self.shared.command = "explore"
            self.shared.command_changed = True

            # Release the shared data
            self.shared.release()
        
#
# Main ROS Thread Code
#
def runros(shared):
    print("Beginning of R")
    # Setup network access for ROS on domain #1.
    os.environ['ROS_LOCALHOST_ONLY']='0'
    os.environ['ROS_DOMAIN_ID']='1'

    # Initialize ROS.
    rclpy.init()

    # Instantiate a simple ROS node, named after the host name, and
    # passing the shared data object.
    node = ROSNode(socket.gethostname(), shared)

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending Run-ROS due to exception: %s" % repr(ex))

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

