#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

#-----FUNCTIONS-----

#Function for recieving the ranges from the laser sensor
def scan_cb(msg):
    global ranges
    ranges =  np.array(msg.ranges)

#Function for recieving the pose data from the IMU
def odom_cb(msg):
    global pose
    pose = msg.pose

#A simple function for movement with variables, linear speed and angular speed
def go(speedlin, speedang):
    twist = Twist()
    twist.linear.x = speedlin
    twist.angular.z = speedang
    pub.publish(twist)

#A lazy function for stopping
def stop():
    go(0,0)

#Utility function to remove clutter for the Pythagorean Theorem
def pathagThero(x):
    return (x[0]**2 + x[1]**2)**.5

#Utility function for calculating the angle between a line and the bot
def calcAng(a, b):
    return np.arctan((b[1] - a[1])/(b[0] - a[0]))

#Utility function that converts polar coordinates to cartesian coordinates
def convert(ang, dis):
    retu = [np.cos(ang) * dis, np.sin(ang) * dis]
    return retu

#A clutter reducing function that gets the angle between the bot and 2 points in the polar coordinate system
def checkAng(ang1, ang2):
    return calcAng(convert(ang1 * np.pi/180, ranges[ang1]), convert(ang2 * np.pi/180, ranges[ang2]))

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

#The realignment function that attempts to reorientate the robot by turning the robot until it is within a certain tolerance
def oriCorrect(radian, tolerance):
    corrected = False
    while not corrected:
        if pose.pose.orientation.z <= radian - tolerance:
            go(0, np.pi/16)
        elif pose.pose.orientation.z >= radian + tolerance:
            go(0, -np.pi/16)
        else:
            stop()
            corrected = True

#Utility function that determines the sign of a value
def signCheck(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    else:
        return -1

#The final return function that reorientates the roomba and moves it back to the center
def goHome():
    oriCorrect(0, 0.01)
    location = [pose.pose.position.x, pose.pose.position.y]
    origin = [0, 0]
    angle = calcAng(location, origin)
    go(0, angle/5)
    rospy.sleep(5)
    stop()
    go((-1) * signCheck(pose.pose.position.x) * pathagThero(location)/10, 0)
    rospy.sleep(10)
    stop()
    stop()
    stop()

#-----CALLS-----
#Creates the name of the node and starts all of the publishers and subscribers
rospy.init_node('pilot')
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_cb)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#-----CODE-----
#Starting Variables
ranges = None
pose = None
rate = rospy.Rate(1)

#A sleep function to give time for the subscribers and getTimes to begin
rospy.sleep(1)

#Starting Variables
start_time = rospy.Time.now()
start_range = ranges
start_pose = pose

#Main code for the Roomba
"""
The robot will move forward at a constant speed for as long as possible until an objects gets within the range of a cone infront of the robot, when that happens
the robot turn left and keep turning until the object it initially got close to is no longer within the cone of the robot. Once the specified time of run has passed
the robot will stop and call the goHome Function, which will realign it so that its orientation is reset, and based of its position turn and go in a straight line
back to the origin. The robot will print out its current task while it is doing it.
""" 
while not rospy.is_shutdown():
    if rospy.Time.now() - start_time < rospy.Duration(secs=60):
        print(">>>>>>>>>>> roomba is vaccuming")
        go(0.2, 0)
        while checkPox(180, .5):
            go(0, np.pi/8)
    else:
        print("<<<<<<<<<< roomba returning to start")
        break
    rate.sleep()
stop()
goHome()
stop()
print("<<<<<<<<<< roomba has arrived home")
stop()