#!/usr/bin/env python
import sys
import time
import rospy
import subprocess
import actionlib

from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def ping_host(host):
    ping_fail_count = rospy.get_param('~ping_fail_count', 2)
    ping_command = "ping -c %s -n -W 1 %s" % (ping_fail_count, host)
    # TODO: don't shell out, use a more secure python library
    p = subprocess.Popen(ping_command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True)
    (output, error) = p.communicate()
    returncode = p.returncode
    return output, error, returncode

class RecorveryController():
    def __init__(self):
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.joy_drive = rospy.Publisher('joy_drive', Joy, queue_size=10)
        self.joy_arm = rospy.Publisher('joy_arm', Joy, queue_size=10)
        self.vel_limit_lost_comms = rospy.Publisher('vel_limit_lost_comms', Int32, queue_size=10)
    
    def working_comms(self):
        working_comms = False
        if (self.ips != "no"):
            for ip in self.ips.split(','):
                (output, error, returncode) = ping_host(ip)
                if returncode == 0:
                    ping = int(output.split('/')[-1].split('.')[0])
                    if ping > 1000:
                        self.vel_limit_lost_comms.publish(0.3)
                    elif ping > 500:
                        self.vel_limit_lost_comms.publish(0.6)
                    elif ping  < 500:
                        self.vel_limit_lost_comms.publish(1)
                    working_comms = True
        else:
            working_comms = True
        return working_comms

    def zero_joystick(self):
        joyDrive = Joy()
        joyArm = Joy()
        if (self.joy_drive_model == 'xbox'):
            joyDrive.axes = [0] * 8
            joyDrive.buttons = [0] * 11
        elif (self.joy_drive_model == 'ec'):
            joyDrive.axes = [0] * 8
            joyDrive.buttons = [0] * 15
        elif (self.joy_drive_model == 'ps5'):
            joyDrive.axes = [0] * 12
            joyDrive.buttons = [0] * 12
        joyArm.axes = [0] * 3
        joyArm.buttons = [0] * 11
        self.joy_drive.publish(joyDrive)
        self.joy_arm.publish(joyArm)

    def do_recovery(self):
        if rospy.is_shutdown(): return
        rospy.logerr('No connection to base station.')
        #if self.connect_to_move_base():
            #if self.goal_in_progress():
                #rospy.loginfo("Navigation in progress, not recovering until finished...")
                #return
            #self.navigation_goal_to(self.recovery_pose)
        self.zero_joystick()
        self.stop_motors()

    def stop_motors(self):
        twist = Twist() # zero motion
        self.cmd_vel.publish(twist)

    def main_loop(self):
        while not rospy.is_shutdown():
            if not self.working_comms():
                self.do_recovery()
            time.sleep(1)


def main():
  rospy.init_node("qr_lost_comms")
  qr_lost_comms = RecorveryController()
  qr_lost_comms.ips = rospy.get_param('~ips_to_monitor')
  qr_lost_comms.joy_drive_model = rospy.get_param('~joy_drive_model')
  rospy.loginfo('Monitoring base station on IP(s): %s.' % qr_lost_comms.ips)
  qr_lost_comms.main_loop() # start monitoring