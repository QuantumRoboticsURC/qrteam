#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from Tkinter import *
import rospy
import time
import os
import serial
import struct
import threading
from std_msgs.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class ArmTeleop:
    def __init__(self):
        ### Initialize serial
        arduino = int(rospy.get_param('~arduino'))
        self.joints_byte_map = {
            'joint5': b"\x04",
            'gripper': b"\x06"
        }
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.Serial = serial.Serial(baudrate=self.baudrate)
        if(arduino != 0):
            self.Serial.port = rospy.get_param("~serial_dev")
            self.Serial.open()
            self.Serial.write(b"\x00")
            self.serial_msg = self.joints_byte_map['joint5'] + struct.pack("<f", 0.0)
            self.serial_msg = self.joints_byte_map['gripper'] + struct.pack("<f", 0.0)
            self.Serial.write(self.serial_msg)
        ### Initialize the publisher for the joints
        self.cmd_vel_mux = rospy.Publisher('cmd_vel_mux', Int32, queue_size=1)
        self.cmd_vel_auto = rospy.Publisher('autonomous/cmd_vel', Twist, queue_size=10)
        self.ejes = rospy.Publisher('arm_teleop/arm_joints', String, queue_size=1)
        self.joint1 = rospy.Publisher('arm_teleop/joint1', Float64, queue_size=1)
        self.joint2 = rospy.Publisher('arm_teleop/joint2', Float64, queue_size=1)
        self.joint3 = rospy.Publisher('arm_teleop/joint3', Float64, queue_size=1)
        self.joint4 = rospy.Publisher('arm_teleop/joint4', Float64, queue_size=1)
        self.joint5 = rospy.Publisher('arm_teleop/joint5', Int32, queue_size=1)
        self.gripper = rospy.Publisher('arm_teleop/gripper', Int32, queue_size=1)
        self.gripper_apertur = 100
        self.joint5_position = 321
        # Initialize the joy subscriber
        self.joy_sub = rospy.Subscriber("joy_arm", Joy, self.on_joy)
        self.joy_abort = rospy.Subscriber("joy_arm", Joy, self.abort_action)
        self.joy_abort.unregister()
        ### Initialize graph interface
        self.ArmControlWindow = Tk()
        self.ArmControlWindow.title("Arm Teleop")
        self.ArmControlWindow.resizable(True, True)
        self.ArmControlWindow.config(cursor="arrow")
        self.root = Frame(self.ArmControlWindow).grid()
        ##### Grpah Interface #####
        #980px width
        #each width 1 of label is 13px
        self.title = Label(self.root, font=("Consolas", 18), width=72, bg="white", bd=0, justify=CENTER)
        self.title.config(text="Arm Teleop")
        self.title.grid(row=0, column=0, columnspan=8, sticky="nsew")
        ##### Section1: When you hold down the button of a joint, the joint moves with the velocity defined in the slider
        self.labelTitleS1 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS1.config(text="Section 1: Move each joint")
        self.labelTitleS1.grid(row=1, column=0, columnspan=4, sticky="nsew")
        self.labelS1 = Label(self.root, font=("Consolas", 10), width=36, bg="white", bd=0, justify=CENTER)
        self.labelS1.config(text="\nHold down a button to move a joint\nthe joint moves with the velocitydefined in the slider\n")
        self.labelS1.grid(row=2, column=0, columnspan=4, sticky="nsew")
        self.labelS1Headers = Label(self.root, font=("Consolas", 8), width=36, bg="white", bd=0, justify=RIGHT, anchor=E)
        self.labelS1Headers.config(text="Joint        |     Velocity      |    Button Clockwise   |Button Counterclockwise")
        self.labelS1Headers.grid(row=3, column=0, columnspan=4, sticky="nsew")
        self.buttonsSection1(1, 4, 0)
        self.S1buttonj1w.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float(self.S1velj1.get()), 1))
        self.S1buttonj1w.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 1))
        self.S1buttonj1c.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float("-"+self.S1velj1.get()), 1))
        self.S1buttonj1c.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 1))
        self.buttonsSection1(2, 5, 0)
        self.S1buttonj2w.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float(self.S1velj2.get()), 2))
        self.S1buttonj2w.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 2))
        self.S1buttonj2c.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float("-"+self.S1velj2.get()), 2))
        self.S1buttonj2c.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 2))
        self.buttonsSection1(3, 6, 0)
        self.S1buttonj3w.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float(self.S1velj3.get()), 3))
        self.S1buttonj3w.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 3))
        self.S1buttonj3c.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float("-"+self.S1velj3.get()) , 3))
        self.S1buttonj3c.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 3))
        self.buttonsSection1(4, 7, 0)
        self.S1buttonj4w.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float(self.S1velj4.get()) , 4))
        self.S1buttonj4w.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 4))
        self.S1buttonj4c.bind("<ButtonPress-1>", lambda event: self.execute_from_graph_interfaceS1(float("-"+self.S1velj4.get()) , 4))
        self.S1buttonj4c.bind("<ButtonRelease-1>", lambda event: self.execute_from_graph_interfaceS1(0.0, 4))
        
        self.S1labelj5 = Label(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=LEFT, anchor=W)
        self.S1labelj5.config(text="Joint 5(servo):")
        self.S1labelj5.grid(row=8, column=0, columnspan=1, sticky="nsew")
        self.S1posj5 = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)
        self.S1posj5.grid(row=8, column=1, columnspan=1, sticky="nsew")
        self.S1posj5.insert(0, "0")
        self.S1buttonj5 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")
        self.S1buttonj5.config(text="Go to the position")
        self.S1buttonj5.grid(row=8, column=2, columnspan=1, sticky="nsew")
        self.S1buttonj5.bind("<Button-1>", lambda event: self.execute_from_graph_interfaceS1(int(self.S1posj5.get()) , 5))

        self.S1labelgripper = Label(self.root, font=("Consolas", 8), width=1, bg="white", bd=0, justify=LEFT, anchor=W)
        self.S1labelgripper.config(text="Gripper: (%open)\n100% open/0% closed")
        self.S1labelgripper.grid(row=9, column=0, columnspan=1, sticky="nsew")
        self.S1aperturag = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)
        self.S1aperturag.grid(row=9, column=1, columnspan=1, sticky="nsew")
        self.S1aperturag.insert(0, "100")
        self.S1buttongripper = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")
        self.S1buttongripper.config(text="Open/Close gripper")
        self.S1buttongripper.grid(row=9, column=2, columnspan=1, sticky="nsew")
        self.S1buttongripper.bind("<Button-1>", lambda event: self.execute_from_graph_interfaceS1(int(self.S1aperturag.get()) , 0))

        ##### Section2: When you hold down the button of a joint, the joint moves with the velocity defined in the slider
        self.labelTitleS2 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS2.config(text="Section 2: Movement preconfigured")
        self.labelTitleS2.grid(row=1, column=4, columnspan=4, sticky="nsew")
        self.labelS2 = Label(self.root, font=("Consolas", 10), width=36, bg="white", bd=0, justify=CENTER)
        self.labelS2.config(text="\nConfigure the movement of the motors\nand press start\n")
        self.labelS2.grid(row=2, column=4, columnspan=4, sticky="nsew")
        self.labelS2Headers = Label(self.root, font=("Consolas", 8), width=36, bg="white", bd=0, justify=LEFT, anchor=W)
        self.labelS2Headers.config(text="       Joint       |      Velocity      |        Time       |       Delay      |")
        self.labelS2Headers.grid(row=3, column=4, columnspan=4, sticky="nsew")
        self.entryandlabelsSection2(1, 4, 4)
        self.entryandlabelsSection2(2, 5, 4)
        self.entryandlabelsSection2(3, 6, 4)
        self.entryandlabelsSection2(4, 7, 4)
        self.S2labelj5 = Label(self.root, font=("Consolas",8), width=1, bg="white", bd=0, justify=LEFT, anchor=W)
        self.S2labelj5.config(text="   Joint 5(servo):")
        self.S2labelj5.grid(row=8, column=4, columnspan=1, sticky="nsew")
        self.S2posj5 = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)
        self.S2posj5.grid(row=8, column=5, columnspan=1, sticky="nsew")
        self.S2posj5.insert(0, "0")
        self.S2delayj5 = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)
        self.S2delayj5.grid(row=8, column=7, columnspan=1, sticky="nsew")
        self.S2delayj5.insert(0, "0.0")
        self.S2labelgripper = Label(self.root, font=("Consolas", 8), width=1, bg="white", bd=0, justify=LEFT, anchor=W)
        self.S2labelgripper.config(text="   Gripper: (%open)\n100% open/0% closed")
        self.S2labelgripper.grid(row=9, column=4, columnspan=1, sticky="nsew")
        self.S2aperturag = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)
        self.S2aperturag.grid(row=9, column=5, columnspan=1, sticky="nsew")
        self.S2aperturag.insert(0, "100")
        self.S2delaygripper = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)
        self.S2delaygripper.grid(row=9, column=7, columnspan=1, sticky="nsew")
        self.S2delaygripper.insert(0, "0.0")

        self.S2startbutton = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")
        self.S2startbutton.config(text="Start")
        self.S2startbutton.grid(row=10, column=5, columnspan=2, sticky="nsew")
        self.S2startbutton.bind("<Button-1>", lambda event: self.execute_from_graph_interfaceS2())
        
        self.joint1.publish(0.0)
        self.joint2.publish(0.0)
        self.joint3.publish(0.0)
        self.joint4.publish(0.0)
        self.joint5.publish(321)
        #self.joint5.publish(0)
        #self.gripper.publish(100)
        ##### --------------- #####
        self.ArmControlWindow.mainloop()

    def buttonsSection1(self, joint, row, col):
        exec('self.S1labelj' + str(joint) + ' = Label(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=LEFT, anchor=W)')
        exec('self.S1labelj' + str(joint) + '.config(text="Joint ' + str(joint) + ':")')
        exec('self.S1labelj' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col) + ', columnspan=1, sticky="nsew")')

        exec('self.S1velj' + str(joint) + ' = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)')
        exec('self.S1velj' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col+1) + ', columnspan=1, sticky="nsew")')
        exec('self.S1velj' + str(joint) + '.insert(0, "0.2")')

        exec('self.S1buttonj' + str(joint) + 'w = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")')
        exec('self.S1buttonj' + str(joint) + 'w.config(text="Clockwise mov")')
        exec('self.S1buttonj' + str(joint) + 'w.grid(row=' + str(row) + ', column=' + str(col+2) + ', columnspan=1, sticky="nsew")')

        exec('self.S1buttonj' + str(joint) + 'c = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")')
        exec('self.S1buttonj' + str(joint) + 'c.config(text="Counterclockwise mov")')
        exec('self.S1buttonj' + str(joint) + 'c.grid(row=' + str(row) + ', column=' + str(col+3) + ', columnspan=1, sticky="nsew")')
    
    def entryandlabelsSection2(self, joint, row, col):
        exec('self.S2labelj' + str(joint) + ' = Label(self.root, font=("Consolas",10), width=1, bg="white", bd=0, justify=LEFT, anchor=W)')
        exec('self.S2labelj' + str(joint) + '.config(text="   Joint ' + str(joint) + ':")')
        exec('self.S2labelj' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col) + ', columnspan=1, sticky="nsew")')

        exec('self.S2velj' + str(joint) + ' = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)')
        exec('self.S2velj' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col+1) + ', columnspan=1, sticky="nsew")')
        exec('self.S2velj' + str(joint) + '.insert(0, "0.0")')

        exec('self.S2timej' + str(joint) + ' = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)')
        exec('self.S2timej' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col+2) + ', columnspan=1, sticky="nsew")')
        exec('self.S2timej' + str(joint) + '.insert(0, "0.0")')

        exec('self.S2delayj' + str(joint) + ' = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)')
        exec('self.S2delayj' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col+3) + ', columnspan=1, sticky="nsew")')
        exec('self.S2delayj' + str(joint) + '.insert(0, "0.0")')
        
    def lock_drive_teleop(self):
        ### phase of lockin_drive_teleop and disable on_joy
        self.joy_sub.unregister() # disable on_joy
        self.joy_abort = rospy.Subscriber("joy_arm", Joy, self.abort_action)
        # Locking drive_teleop with cmd_vel_mux
        self.cmd_vel_mux.publish(1)
        twist = Twist()
        self.cmd_vel_auto.publish(twist)
    
    def unlock_drive_teleop(self):
        ### phase of unlock_drive_teleop and enable on_joy
        self.cmd_vel_mux.publish(0)
        self.joy_abort.unregister()
        self.joy_sub = rospy.Subscriber("joy_arm", Joy, self.on_joy)

    def abort_action(self, data):
        # button 0 to kill the action and restart the teleop
        try:
            if data.buttons[5] == 1:
                os.system("rosnode kill /sar_arm_velnopos_graph")
                self.ArmControlWindow.destroy()
        except:
            if data == 0:
                self.joint1.publish(0.0)
                self.joint2.publish(0.0)
                self.joint3.publish(0.0)
                self.joint4.publish(0.0)
                os.system("rosnode kill /sar_arm_velnopos_graph")
                self.ArmControlWindow.destroy()

    def on_joy(self, data):
        self.lock_drive_teleop()
        ### BTN 0:
        if data.axes[1] != 0 and data.buttons[2] == 1:
            self.joint4.publish(data.axes[1])
            self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: " + str(data.axes[1]) + ", j5: " + str(self.joint5_position) + ", g: " + str(self.gripper_apertur)
            self.ejes.publish(self.ejest)
        if data.axes[1] == 0:
            self.joint4.publish(0.0)
            self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: " + str(self.joint5_position) + ", g: " + str(self.gripper_apertur)
            self.ejes.publish(self.ejest)
        if data.buttons[1] == 1: #data.axes[0] != 0 and 
            if data.axes[0] > 0:
                self.joint5_position -= int(data.axes[0]*20)
                if self.joint5_position < 0:
                    self.joint5_position = 0
                self.joint5.publish(self.joint5_position)
                self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: " + str(self.joint5_position) + ", g: " + str(self.gripper_apertur)
            elif data.axes[0] < 0:
                self.joint5_position -= int(data.axes[0]*20)
                if self.joint5_position > 642:
                    self.joint5_position = 642
                self.joint5.publish(self.joint5_position)
                self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: " + str(self.joint5_position) + ", g: " + str(self.gripper_apertur)
        if data.buttons[0] == 1: #data.axes[0] != 0
            if data.axes[0] > 0:
                self.gripper_apertur += int(data.axes[0]*10)
                if self.gripper_apertur > 100:
                    self.gripper_apertur = 100
                self.gripper.publish(self.gripper_apertur)
                self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: " + str(self.joint5_position) + ", g: " + str(self.gripper_apertur)
            elif data.axes[0] < 0:
                self.gripper_apertur += int(data.axes[0]*10)
                if self.gripper_apertur < 0:
                    self.gripper_apertur = 0
                self.gripper.publish(self.gripper_apertur)
                self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: " + str(self.joint5_position) + ", g: " + str(self.gripper_apertur)
        self.unlock_drive_teleop() # unlock drive_teleop
        
		
    def execute_from_graph_interfaceS1(self, data, joint):
        ### phase of lockin_drive_teleop and disable on_joy
        self.lock_drive_teleop()
        ### phase of send the data of the joints
        if(joint == 0):
            self.gripper_apertur = int(data)
            self.gripper.publish(self.gripper_apertur)
            self.serial_msg = self.joints_byte_map['gripper'] + struct.pack("<f", 0.0)
            self.Serial.write(self.serial_msg)
            self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: ?, g: " + str(data)
        if(joint == 1):
            self.joint1.publish(round(data, 2))
            if(data != 0):
                self.S1labelj1.config(bg="#34eb61")
                self.S1labelj1.config(text="Joint 1: moving")
            else:
                self.S1labelj1.config(bg="white")
                self.S1labelj1.config(text="Joint 1: ")
            self.ejest = "j1: " + str(data) + ", j2: 0.0, j3: 0.0, j4: 0.0, j5: ?, g: ?"
        elif(joint == 2):
            self.joint2.publish(round(data, 2))
            if(data != 0):
                self.S1labelj2.config(bg="#34eb61")
                self.S1labelj2.config(text="Joint 2: moving")
            else:
                self.S1labelj2.config(bg="white")
                self.S1labelj2.config(text="Joint 2: ")
            self.ejest = "j1: 0.0, j2: " + str(data) + ", j3: 0.0, j4: 0.0, j5: ?, g: ?"
        elif(joint == 3):
            self.joint3.publish(round(data, 2))
            if(data != 0):
                self.S1labelj3.config(bg="#34eb61")
                self.S1labelj3.config(text="Joint 3: moving")
            else:
                self.S1labelj3.config(bg="white")
                self.S1labelj3.config(text="Joint 3: ")
            self.ejest = "j1: 0.0, j2: 0.0, j3: " + str(data) + ", j4: 0.0, j5: ?, g: ?"
        elif(joint == 4):
            self.joint4.publish(round(data, 2))
            if(data != 0):
                self.S1labelj4.config(bg="#34eb61")
                self.S1labelj4.config(text="Joint 4: moving")
            else:
                self.S1labelj4.config(bg="white")
                self.S1labelj4.config(text="Joint 4: ")
            self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: " + str(data) + ", j5: ?, g: ?"
        elif(joint == 5):
            self.joint5_position = int(data)
            self.joint5.publish(self.joint5_position)
            self.serial_msg = self.joints_byte_map['joint5'] + struct.pack("<f", 0.0)
            self.Serial.write(self.serial_msg)
            self.ejest = "j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: " + str(data) + ", g: ?"
        self.ejes.publish(self.ejest)
        rospy.loginfo(self.ejest)
        ### phase of unlock_drive_teleop and enable on_joy
        self.unlock_drive_teleop()
    
    def execute_from_graph_interfaceS2(self):
        self.S2startbutton.config(bg="#34eb61")
        self.S2startbutton.config(text="Executing/Cancel")
        self.S2startbutton.bind("<Button-1>", lambda event: self.abort_action(0))
        thread_exec = threading.Thread(target=self.thread_execute_from_graph_interfaceS2)
        thread_exec.setDaemon(True)
        thread_exec.start()

    def thread_execute_from_graph_interfaceS2(self):        
        ### phase of lockin_drive_teleop and disable on_joy
        self.lock_drive_teleop()
        ### phase of send the data of the joints
        gripper = "?"
        self.joint1.publish(0.0)
        j1 = "0.0"
        self.joint2.publish(0.0)
        j2 = "0.0"
        self.joint3.publish(0.0)
        j3 = "0.0"
        self.joint4.publish(0.0)
        j4 = "0.0"
        j5 = "?"
        self.ejest = "j1: " + j1 + ", j2: " + j2 + ", j3: " + j3 + ", j4: " + j4 + ", j5: " + j5 + ", g: " + gripper
        self.ejes.publish(self.ejest)
        rospy.loginfo(self.ejest)
        
        ###### Secuence of the joints based on the data of the graph interface ######
        self.delays = [float(self.S2delaygripper.get()), float(self.S2delayj1.get()), float(self.S2delayj2.get()), float(self.S2delayj3.get()), float(self.S2delayj4.get()), float(self.S2delayj5.get())]
        self.joints = [float(self.S2aperturag.get()), float(self.S2velj1.get()), float(self.S2velj2.get()), float(self.S2velj3.get()), float(self.S2velj4.get()), float(self.S2posj5.get())]
        self.jtimes = [0, float(self.S2timej1.get()), float(self.S2timej2.get()), float(self.S2timej3.get()), float(self.S2timej4.get()), 0]
        self.executed = [0,0,0,0,0,0]
        timestart = time.time()

        # Get the largest time of the joints
        self.maxtimes = [self.delays[0] + self.jtimes[0], self.delays[1] + self.jtimes[1], self.delays[2] + self.jtimes[2], self.delays[3] + self.jtimes[3], self.delays[4] + self.jtimes[4], self.delays[5] + self.jtimes[5]]
        maxtime = max(self.maxtimes) + 0.1
        timeelapsed = 0
        while (timeelapsed < maxtime):
            timeelapsed = time.time() - timestart
            for i in range(len(self.delays)):
                if (timeelapsed >= self.delays[i]):
                    if (self.executed[i] == 0):
                        if i == 0:
                            self.gripper_apertur = self.joints[i]
                            self.gripper.publish(self.gripper_apertur)
                            gripper = self.gripper_apertur
                            self.executed[i] = 3
                        elif i == 1:
                            self.joint1.publish(self.joints[i])
                            j1 = self.joints[i]
                            timestart_j1 = time.time()
                            self.executed[i] = 1
                        elif i == 2:
                            self.joint2.publish(self.joints[i])
                            j2 = self.joints[i]
                            timestart_j2 = time.time()
                            self.executed[i] = 1
                        elif i == 3:
                            self.joint3.publish(self.joints[i])
                            j3 = self.joints[i]
                            timestart_j3 = time.time()
                            self.executed[i] = 1
                        elif i == 4:
                            self.joint4.publish(self.joints[i])
                            j4 = self.joints[i]
                            timestart_j4 = time.time()
                            self.executed[i] = 1
                        elif i == 5:
                            self.joint5_position = self.joints[i]
                            self.joint5.publish(self.joint5_position)
                            j5 = self.joint5_position
                            self.executed[i] = 3
                    elif (self.executed[i] == 2):
                        if i == 1:
                            if (time.time() - timestart_j1) >= self.jtimes[i]:
                                self.joint1.publish(0.0)
                                j1 = "0.0"
                                self.executed[i] = 3
                        elif i == 2:
                            if (time.time() - timestart_j2) >= self.jtimes[i]:
                                self.joint2.publish(0.0)
                                j2 = "0.0"
                                self.executed[i] = 3
                        elif i == 3:
                            if (time.time() - timestart_j3) >= self.jtimes[i]:
                                self.joint3.publish(0.0)
                                j3 = "0.0"
                                self.executed[i] = 3
                        elif i == 4:
                            if (time.time() - timestart_j4) >= self.jtimes[i]:
                                self.joint4.publish(0.0)
                                j4 = "0.0"
                                self.executed[i] = 3
            # search 1 or 3 in self.executed
            if(1 in self.executed or 3 in self.executed):
                for i in range(len(self.executed)):
                    if(self.executed[i] == 1):
                        self.executed[i] = 2
                    elif(self.executed[i] == 3):
                        self.executed[i] = 4
                self.ejest = "j1: " + str(j1) + ", j2: " + str(j2) + ", j3: " + str(j3) + ", j4: " + str(j4) + ", j5: " + str(j5) + ", g: " + str(gripper)
                self.ejes.publish(self.ejest)
                rospy.loginfo(self.ejest)
        self.joint1.publish(0.0)
        j1 = "0.0"
        self.joint2.publish(0.0)
        j2 = "0.0"
        self.joint3.publish(0.0)
        j3 = "0.0"
        self.joint4.publish(0.0)
        j4 = "0.0"
        self.ejest = "j1: " + str(j1) + ", j2: " + str(j2) + ", j3: " + str(j3) + ", j4: " + str(j4) + ", j5: " + str(j5) + ", g: " + str(gripper)
        self.ejes.publish(self.ejest)
        rospy.loginfo(self.ejest)
        ###### --------------------------------------------------------------- ######
        ### phase of unlock_drive_teleop and enable on_joy
        self.unlock_drive_teleop()
        self.S2startbutton.config(bg="#ff523b")
        self.S2startbutton.config(text="Start")
        self.S2startbutton.bind("<Button-1>", lambda event: self.execute_from_graph_interfaceS2())


def main():
	rospy.init_node("sar_arm_velnopos_graph")
	sar_base_arm_test = ArmTeleop()
	rospy.spin()