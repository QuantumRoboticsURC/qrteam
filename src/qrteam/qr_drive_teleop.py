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
        self.vel_limit_setting = 1 # default to medium speed
        # Initialize publishers
        self.cmd_vel_teleop = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_mux = rospy.Publisher("cmd_vel_mux", Int32, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy_drive", Joy, self.on_joy)
        self.vel_limit_lost_comms = rospy.Subscriber("vel_limit_lost_comms", Int32, self.on_vel_limit_lost_comms)
        self.left_speed = 0
        self.right_speed = 0
        self.last_axis_left = 0
        self.last_axis_right = 0
        self.acel = 10
        self.desacel = 0.5

    def on_vel_limit_lost_comms(self, msg):
        self.vel_limit_setting = msg.data

    def on_joy(self, data):
        self.joy_sub.unregister()
        if (self.joy_drive_model == "ps5"):
            # Set speed ratio using d-pad
            if data.axes[7] == 1: # full speed (d-pad up)
                self.vel_limit_setting = 1
            if data.axes[6] != 0: # medium speed (d-pad left or right)
                self.vel_limit_setting = 0.6
            if data.axes[7] == -1: # low speed (d-pad down)
                self.vel_limit_setting = 0.3
            # Move base mode
            if data.buttons[9]: # Select button
                self.cmd_vel_mux.publish(2)
                time.sleep(3)
                self.cmd_vel_mux.publish(0)
            # Autonomous mode
            if data.buttons[10]: # Start button
                self.cmd_vel_mux.publish(1)
                time.sleep(3)
                self.cmd_vel_mux.publish(0)
            # Drive sticks
            ##### left speed axis 1
            axis_left = data.axes[1]
            if abs(axis_left) == 0 and self.left_speed != 0:
                if abs(self.left_speed) < self.desacel: self.left_speed = 0
                elif self.left_speed >= self.desacel: self.left_speed -= self.desacel
                elif self.left_speed <= -self.desacel: self.left_speed += self.desacel
            else:
                left_difference = axis_left-self.left_speed
                if (left_difference > 0 and self.left_speed < self.vel_limit_setting):
                    self.left_speed += abs(axis_left) / self.acel
                    if self.left_speed > 1: self.left_speed = 1
                elif (left_difference < 0 and self.left_speed > -self.vel_limit_setting):
                    self.left_speed -= abs(axis_left) / self.acel
                    if self.left_speed < -1: self.left_speed = -1
            ##### right speed axis 5
            axis_right = data.axes[5]
            if abs(axis_right) == 0 and self.right_speed != 0:
                if abs(self.right_speed) < self.desacel: self.right_speed = 0
                elif self.right_speed >= self.desacel: self.right_speed -= self.desacel
                elif self.right_speed <= -self.desacel: self.right_speed += self.desacel
            else:
                right_difference = axis_right-self.right_speed
                if (right_difference > 0 and self.right_speed < self.vel_limit_setting):
                    self.right_speed += abs(axis_right) / self.acel
                    if self.right_speed > 1: self.right_speed = 1
                elif (right_difference < 0 and self.right_speed > -self.vel_limit_setting):
                    self.right_speed -= abs(axis_right) / self.acel
                    if self.right_speed < -1: self.right_speed = -1

        elif (self.joy_drive_model == "ec"):
            # Set speed ratio using d-pad
            if data.axes[7] == 1: # full speed (d-pad up)
                self.speed_setting = 1
            if data.axes[6] != 0: # medium speed (d-pad left or right)
                self.speed_setting = 2
            if data.axes[7] == -1: # low speed (d-pad down)
                self.speed_setting = 3
            # Move base mode
            if data.buttons[10]: # Select button
                self.cmd_vel_mux.publish(2)
                time.sleep(3)
                self.cmd_vel_mux.publish(0)
            # Autonomous mode
            if data.buttons[11]: # Start button
                self.cmd_vel_mux.publish(1)
                time.sleep(3)
                self.cmd_vel_mux.publish(0)
            # Drive sticks
            ##### left speed axis 1
            axis_left = data.axes[1]
            if abs(axis_left) == 0 and self.left_speed != 0:
                if abs(self.left_speed) < self.desacel: self.left_speed = 0
                elif self.left_speed >= self.desacel: self.left_speed -= self.desacel
                elif self.left_speed <= -self.desacel: self.left_speed += self.desacel
            else:
                left_difference = axis_left-self.left_speed
                if (left_difference > 0 and self.left_speed < self.vel_limit_setting):
                    self.left_speed += abs(axis_left) / self.acel
                    if self.left_speed > 1: self.left_speed = 1
                elif (left_difference < 0 and self.left_speed > -self.vel_limit_setting):
                    self.left_speed -= abs(axis_left) / self.acel
                    if self.left_speed < -1: self.left_speed = -1
            rospy.loginfo("left_speed: %f" % self.left_speed)
            ##### right speed axis 3
            axis_right = data.axes[3]
            if abs(axis_right) == 0 and self.right_speed != 0:
                if abs(self.right_speed) < self.desacel: self.right_speed = 0
                elif self.right_speed >= self.desacel: self.right_speed -= self.desacel
                elif self.right_speed <= -self.desacel: self.right_speed += self.desacel
            else:
                right_difference = axis_right-self.right_speed
                if (right_difference > 0 and self.right_speed < self.vel_limit_setting):
                    self.right_speed += abs(axis_right) / self.acel
                    if self.right_speed > 1: self.right_speed = 1
                elif (right_difference < 0 and self.right_speed > -self.vel_limit_setting):
                    self.right_speed -= abs(axis_right) / self.acel
                    if self.right_speed < -1: self.right_speed = -1
            rospy.loginfo("right_speed: %f" % self.right_speed)

        elif (self.joy_drive_model == "xbox"):
            # Set speed ratio using d-pad
            if data.axes[7] == 1: # full speed (d-pad up)
                self.speed_setting = 1
            if data.axes[6] != 0: # medium speed (d-pad left or right)
                self.speed_setting = 2
            if data.axes[7] == -1: # low speed (d-pad down)
                self.speed_setting = 3
            # Move base mode
            if data.buttons[6]: # Select button
                self.cmd_vel_mux.publish(2)
                time.sleep(3)
                self.cmd_vel_mux.publish(0)
            # Autonomous mode
            if data.buttons[7]: # Start button
                self.cmd_vel_mux.publish(1)
                time.sleep(3)
                self.cmd_vel_mux.publish(0)
            # Drive sticks
            ##### left speed axis 1
            axis_left = data.axes[1]
            if abs(axis_left) == 0 and self.left_speed != 0:
                if abs(self.left_speed) < self.desacel: self.left_speed = 0
                elif self.left_speed >= self.desacel: self.left_speed -= self.desacel
                elif self.left_speed <= -self.desacel: self.left_speed += self.desacel
            else:
                left_difference = axis_left-self.left_speed
                if (left_difference > 0 and self.left_speed < self.vel_limit_setting):
                    self.left_speed += abs(axis_left) / self.acel
                    if self.left_speed > 1: self.left_speed = 1
                elif (left_difference < 0 and self.left_speed > -self.vel_limit_setting):
                    self.left_speed -= abs(axis_left) / self.acel
                    if self.left_speed < -1: self.left_speed = -1
            rospy.loginfo("left_speed: %f" % self.left_speed)
            ##### right speed axis 4
            axis_right = data.axes[4]
            if abs(axis_right) == 0 and self.right_speed != 0:
                if abs(self.right_speed) < self.desacel: self.right_speed = 0
                elif self.right_speed >= self.desacel: self.right_speed -= self.desacel
                elif self.right_speed <= -self.desacel: self.right_speed += self.desacel
            else:
                right_difference = axis_right-self.right_speed
                if (right_difference > 0 and self.right_speed < self.vel_limit_setting):
                    self.right_speed += abs(axis_right) / self.acel
                    if self.right_speed > 1: self.right_speed = 1
                elif (right_difference < 0 and self.right_speed > -self.vel_limit_setting):
                    self.right_speed -= abs(axis_right) / self.acel
                    if self.right_speed < -1: self.right_speed = -1
            rospy.loginfo("right_speed: %f" % self.right_speed)

        # Convert skid steering speeds to twist speeds
        self.linear_vel  = (self.left_speed + self.right_speed) / 2.0 # (m/s)
        self.angular_vel  = (self.left_speed - self.right_speed) / 2.0 # (rad/s)

        # Publish Twist
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = self.angular_vel
        self.cmd_vel_teleop.publish(twist)

        self.joy_sub = rospy.Subscriber("joy_drive", Joy, self.on_joy)

def main():
    rospy.init_node("qr_drive_teleop")
    qr_drive_teleop = DriveTeleop()
    qr_drive_teleop.joy_drive_model = rospy.get_param('~joy_drive_model')
    rospy.spin()