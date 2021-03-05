#!/usr/bin/env python

#Imports
import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

#Variables
global Zztemp
global Base_lin
global Base_ang
global lin
global ang

#fill in scan callback
#Function receives and processes the ranges from the distance scanner
def scan_cb(msg):
    global ranges
    ranges =  np.array(msg.ranges)

#it is not necessary to add more code here but it could be useful
#Function receives and processes the keys from the keyboards
def key_cb(msg):
    global state; global last_key_press_time
    state = msg.data
    last_key_press_time = rospy.Time.now()

#odom is also not necessary but very useful
#Function receives and processes the odometry data from the IMU
def odom_cb(msg):
    global pose
    pose = msg.pose

#A lazy function for stopping
def stop():
    go(0,0)

#print the state of the robot
def print_state():
   print("---")
   print("STATE: " + state)
   print("CURRENT LINEAR SPEED: " + str(lin) + " m/s")
   print("CURRENT ANGULAR SPEED: " + str(ang) + " rad/s")

   # calculate time since last key stroke
   time_since = rospy.Time.now() - last_key_press_time
   print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

#A simple function for movement with variables, linear speed and angular speed
def go(speedlin, speedang):
    twist = Twist()
    twist.linear.x = speedlin
    twist.angular.z = speedang
    cmd_vel_pub.publish(twist)

#Checks if the robot has something in its path in a cone shape in front of it, and returns a boolean based off of the information given
def checkPox(cone, margin):
    retu = False
    for x in range(360 - cone/2 -1, 360):
        if ranges[x] <= margin:
            retu = True
    for x in range(0, cone/2):
        if ranges[x] <= margin:
            retu = True
    return retu

#A function that makes the robot zig-zag around by periodically making it turn around
def Zig_zagging(Zz):
    Zz *= -1
    go(0, Zz * np.pi/2)
    rospy.sleep(1)
    go(lin, ang)
    return Zz

#init node
rospy.init_node('dancer')

#subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

#calls and connects all of the publishers and subscribers
key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

#start in state halted and grab the current time
rospy.sleep(1)
print"---Beginning Run---"
state = "H"
last_key_press_time = rospy.Time.now()

#set rate
rate = rospy.Rate(10)

#Setup varibles and the start state of the robot
start_time = rospy.Time.now()
#---Zig-zagging variables---
Zig_zag = False
Zztemp = -1
Zzstart = rospy.Time.now()
#---Spiraling variables---
spiral = False
spiral_ang = 0.3
sptemp = False
spstart = rospy.Time.now()
#---Normal movement variables---
lin = 0
ang = 0
base = [lin, ang]

#Wait for published topics, exit on ^c
while not rospy.is_shutdown():
    #Checks if the robot is too close to an object
    if checkPox(360, .2):
        stop()
        Zig_zag = False
        spiral = False
        lin = 0
        ang = 0
        state = "H"
    #Checks and makes the robot zig-zag around
    if Zig_zag and rospy.Time.now() - Zzstart >= rospy.Duration(secs=3):
        Zztemp = Zig_zagging(Zztemp)
        Zzstart = rospy.Time.now()
    #Checks and makes the robot move in a spiraing movement
    if spiral:
        #The forward acceleration part of the spiral motion
        if sptemp and rospy.Time.now() - spstart >= rospy.Duration(secs=10):
            ang -= spiral_ang
            lin += 0.1
            spstart = rospy.Time.now()
            sptemp = not sptemp
        #The normal spiral motion
        if not sptemp and rospy.Time.now() - spstart >= rospy.Duration(secs=2):
            lin -= 0.1
            ang += spiral_ang
            spstart = rospy.Time.now()
            sptemp = not sptemp
    #Basic movements
    if state == "f":
        lin += 0.1
    elif state == "b":
        lin += -0.1
    elif state == "l":
        ang += np.pi/10
    elif state == "r":
        ang += -np.pi/10
    #Full stop operations
    elif state == "h":
        stop()
        Zig_zag = False
        spiral = False
        lin = 0
        ang = 0
    #Actives the spiraling movement patterns
    elif state == "s":
        spiral = not spiral
        if spiral:
            base = [lin, ang]
            lin += 0.1
            ang += spiral_ang
        else:
            lin = base[0]
            ang = base[1]
    #Actives the zig-zagging movement patterns
    elif state == "z":
        Zig_zag = not Zig_zag
        Zztemp *= -1
        go(0, Zztemp * np.pi/4)
        rospy.sleep(1)
        go(lin, ang)
        start_time = rospy.Time.now()
    
    #Processing and issues the movement orders
    go(lin, ang)
    state = state.upper()
    print_state()
    rate.sleep()