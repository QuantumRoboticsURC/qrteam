#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Int64

class CmdVelMux:
    def __init__(self):
        self.cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.led_matrix = rospy.Publisher("matrix",Int64,queue_size = 10)
        self.cmd_vel_mux = rospy.Subscriber("cmd_vel_mux", Int32, callback=self.muxer)
        self.cmd_vel_auto = rospy.Subscriber("autonomous/cmd_vel", Twist, callback=self.autonomous_cmd_vel)
        self.cmd_vel_teleop = rospy.Subscriber("teleop/cmd_vel", Twist, callback=self.teleop_cmd_vel)
        self.cmd_vel_movebase = rospy.Subscriber("move_base/cmd_vel", Twist, callback=self.move_base_cmd_vel)
        self.mode = 0

    def teleop_cmd_vel(self, twist_teleop):
        if (self.mode == 0):
            self.cmd_vel.publish(twist_teleop)

    def autonomous_cmd_vel(self, twist_auto):
        if (self.mode == 1):
            self.cmd_vel.publish(twist_auto)
    
    def move_base_cmd_vel(self, twist_movebase):
        if (self.mode == 2):
            self.cmd_vel.publish(twist_movebase)

    def muxer(self, mode_cmd_vel_mux):
        self.mode = mode_cmd_vel_mux.data
        # Turn on led matrix to indicate mode
        if (self.mode == 0):
            self.led_matrix.publish(1)
        elif (self.mode == 1):
            self.led_matrix.publish(0)
        elif (self.mode == 2):
            self.led_matrix.publish(0)

def main():
    rospy.init_node("qr_cmd_vel_mux")
    qr_cmd_vel_mux = CmdVelMux()
    rospy.spin()
