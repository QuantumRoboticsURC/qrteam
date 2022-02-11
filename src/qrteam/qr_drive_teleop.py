#!/usr/bin/python

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class DriveTeleop:
    def __init__(self):
        # Initialize speed setting
        self.speed_setting = 2 # default to medium speed
        # Initialize publishers
        self.cmd_vel_teleop = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_mux = rospy.Publisher("cmd_vel_mux", Int32, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy_drive", Joy, self.on_joy)

    def on_joy(self, data):
        self.joy_sub.unregister()
        # Set speed ratio using d-pad
        if data.axes[7] == 1: # full speed (d-pad up)
            self.speed_setting = 1
        if data.axes[6] != 0: # medium speed (d-pad left or right)
            self.speed_setting = 2
        if data.axes[7] == -1: # low speed (d-pad down)
            self.speed_setting = 3

        # Drive sticks
        left_speed = -data.axes[1] / self.speed_setting # left stick
        right_speed = -data.axes[5] / self.speed_setting # right stick

        # Convert skid steering speeds to twist speeds
        linear_vel  = (left_speed + right_speed) / 2.0 # (m/s)
        angular_vel  = (left_speed - right_speed) / 2.0 # (rad/s)

        # Publish Twist
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_teleop.publish(twist)

        # Move base mode
        if data.buttons[9]: # Options button
            self.cmd_vel_mux.publish(2)
            time.sleep(3)
            self.cmd_vel_mux.publish(0)
        # Autonomous mode
        if data.buttons[10]: # Start button
            self.cmd_vel_mux.publish(1)
            time.sleep(3)
            self.cmd_vel_mux.publish(0)
        self.joy_sub = rospy.Subscriber("joy_drive", Joy, self.on_joy)

def main():
    rospy.init_node("qr_drive_teleop")
    qr_drive_teleop = DriveTeleop()
    rospy.spin()