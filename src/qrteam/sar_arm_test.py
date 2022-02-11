#!/usr/bin/python

# from sympy import true
import rospy
import time
import os
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class ArmTeleop:
	def __init__(self):
		# Initialize the publisher for the joints
		self.cmd_vel_mux = rospy.Publisher('cmd_vel_mux', Int32, queue_size=1)
		self.cmd_vel_auto = rospy.Publisher('autonomous/cmd_vel', Twist, queue_size=10)
		self.eje1 = rospy.Publisher('arm_teleop/arm_1_joint', Int32, queue_size=1)
		self.eje2 = rospy.Publisher("arm_teleop/arm_2_joint", Int32, queue_size=1)
		self.eje3 = rospy.Publisher("arm_teleop/arm_3_joint", Int32, queue_size=1)
		self.eje4 = rospy.Publisher("arm_teleop/arm_4_joint", Int32, queue_size=1)
		self.eje5 = rospy.Publisher("arm_teleop/arm_5_joint", Int32, queue_size=1)
		# Initialize the publisher for the gripper
		self.gripper = rospy.Publisher('arm_teleop/gripper_arm', Int32, queue_size=1)
		# Initialize the joy subscriber
		self.joy_sub = rospy.Subscriber("joy_arm", Joy, self.on_joy)
		self.joy_abort = rospy.Subscriber("joy_arm", Joy, self.abord_action)
		self.joy_abort.unregister()

	def go_home(self):
		self.joy_abort = rospy.Subscriber("joy_arm", Joy, self.abord_action)
		# Locking drive_teleop with cmd_vel_mux
		self.cmd_vel_mux.publish(1)
		twist = Twist()
		self.cmd_vel_auto.publish(twist)
		# Go to home position
		self.eje1.publish(0)
		self.eje2.publish(0)
		self.eje3.publish(-170)
		self.eje4.publish(160)
		self.eje5.publish(0)
		self.gripper.publish(0)
		# Wait for the joints to be evaluated
		while True:
			# leer los grados del joint 1, joint 2, joint 3, joint 4, joint 5 y gripper
			# si todos los grados son 0, terminar
			break

	def unlock_drive_teleop(self):
		self.cmd_vel_mux.publish(0)

	def on_joy(self, data):
		self.joy_sub.unregister()
		# Call the global lock variable to prevent multiple callbacks
		# Switch case for the joy buttons
		
		### BTN 0: Home
		if data.buttons[0] == 1:
			# Home
			self.go_home()
			time.sleep(1) # This line will be removed when go_home() evaluates the joints positions
			self.unlock_drive_teleop()
		
		### BTN 1: Action 1d [Pull object by a rope]
		if data.buttons[1] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Pull object by a rope
			self.Action1d()
			self.unlock_drive_teleop()

		### BTN 2: Action 1b [Take a supply containers]
		if data.buttons[2] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Take a supply containers
			self.Action1b()
			self.unlock_drive_teleop()
		
		### BTN 3: Action 1a [Take a screwdriver]
		if data.buttons[3] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Take a screwdriver
			self.Action1a()
			self.unlock_drive_teleop()

		### BTN 4: Action 2c [Take a rock]
		if data.buttons[4] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Take a rock
			self.Action2c()
			self.unlock_drive_teleop()
		
		### BTN 5: Action 2a [Take a cache container and save it]
		elif data.buttons[5] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Save the cache container
			self.Action2a()
			self.unlock_drive_teleop()
		
		### BTN 6: Action 2b [Undo a latch on a hinged panel]
		elif data.buttons[6] == 1:
			# Home
			self.go_home()
			time.sleep(1)
			# Undo a latch on a hinged panel
			self.Action2b()
			self.unlock_drive_teleop()
		
		### BTN 7: Action 2c [Write on a keybord]
		elif data.buttons[7] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Write on a keybord
			self.Action2c()
			self.unlock_drive_teleop()
		
		### BTN 8: Action 2d [Move a joystick]
		if data.buttons[8] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Move a joystick
			self.Action2d()
			self.unlock_drive_teleop()

		### BTN 9: Action 2e [Take and insert a USB]
		if data.buttons[9] == 1:
			# Home
			self.go_home()
			time.sleep(1) # Temporary line
			# Take and insert a USB
			self.Action2e()
			self.unlock_drive_teleop()

		### BTN 10: Action 2f [Buttons, switches and joysticks]
		if data.buttons[10] == 1:
			# Home
			self.go_home()
			time.sleep(1) # This is the end of the file
			# Buttons, switches and joysticks
			self.Action2f()
			self.unlock_drive_teleop()
		self.joy_abort.unregister()
		self.joy_sub = rospy.Subscriber("joy_arm", Joy, self.on_joy)

	def abord_action(self, data):
		### BTN 0: Home
		if data.buttons[0] == 1:
			os.system("rosnode kill sar_arm_test")

	def Action1a(self): # Take a screwdriver
		return 0

	def Action1b(self): # Take a supply containers
		return 0

	def Action1c(self): # Take a rock
		return 0

	def Action1d(self): # Pull object by a rope
		return 0

	def Action2a(self): # Save the cache container
		return 0

	def Action2b(self): # Undo a latch on a hinged panel
		return 0

	def Action2c(self): # Write on a keybord
		return 0

	def Action2d(self): # Move a joystick
		return 0

	def Action2e(self): # Take and insert a USB
		return 0

	def Action2f(self): # Buttons, switches and joysticks
		return 0

def main():
	rospy.init_node("sar_arm_test")
	sar_arm_test = ArmTeleop()
	rospy.spin()