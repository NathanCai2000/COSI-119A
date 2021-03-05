#!/usr/bin/env python

#Imports
import rospy
import sys
import math
import tf
import random
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

#odom is also not necessary but very useful
#Function receives and processes the odometry data from the IMU
def odom_cb(msg):
    global pose
    pose = msg.pose

#A lazy function for stopping
def stop():
    go(0,0)

#Utility function that converts polar coordinates to cartesian coordinates
def convert(ang, dis):
    retu = [np.cos(ang) * dis, np.sin(ang) * dis]
    return retu

#Utility function for calculating the angle between a line and the bot
def calcAng(a, b):
    return np.arctan((b[1] - a[1])/(b[0] - a[0]))

#A clutter reducing function that gets the angle between the bot and 2 points in the polar coordinate system
def checkAng(ang1, ang2):
    return calcAng(convert(ang1 * np.pi/180, ranges[ang1]), convert(ang2 * np.pi/180, ranges[ang2]))

#Utility function that determines the sign of a value
def signCheck(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    else:
        return -1

#A simple function for movement with variables, linear speed and angular speed
def go(speedlin, speedang):
    twist = Twist()
    twist.linear.x = speedlin
    twist.angular.z = speedang
    cmd_vel_pub.publish(twist)

#Checks if the robot has something in its path in a cone shape in front of it, and returns a boolean based off of the information given
def checkPox(margin, cone=45):
    retu = False
    for x in range(0, cone):
        if ranges[x] <= margin:
            retu = True
    for x in range(360 - cone, 360):
        if ranges[x] <= margin:
            retu = True
    return retu

def correctPox(margin, side=0):
    retu = 0
    temp = [ranges[0:135], ranges[225:360]]
    base_Sign = 1
    if side == 1:
        base_Sign = -1
    if min(temp[side]) <= margin:
        retu = 2* margin / min(temp[side]) * base_Sign
    else:
        retu = min(temp[side]) / (2* margin * base_Sign)
    return retu

def wall_Ahead(margin, side=0):
    retu = 0
    temp = [ranges[0:10], ranges[350:360]]
    base_Sign = 1
    if side == 1:
        base_Sign *= -1
    if min(temp[side]) <= margin:
        retu = 2* margin / min(temp[side]) * base_Sign
    else:
        retu = min(temp[side]) / (2* margin * base_Sign)
    return retu

def getAlignment():
    retu = [0, 0]
    temp1 = []
    temp2 = []
    for x in range(0, len(ranges)/2, 2):
        temp1.append(checkAng(x, x + 1))
    retu[0] = sum(temp1)/len(temp1)
    for x in range(len(ranges)/2, len(ranges), 2):
        temp2.append(checkAng(x, x + 1))
    retu[1] = sum(temp2)/len(temp2)
    retu = [retu[0]*np.pi/180, retu[1]*np.pi/180]
    return retu #[0] Right Side, [1] Left Side

def clean(minV, maxV, data):
    retu = data
    for x in range(len(retu)):
        retu[x] = min(max(minV, retu[x]), maxV)
    return retu
    

#init node
rospy.init_node('dancer')

#subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

#calls and connects all of the publishers and subscribers
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

rospy.sleep(1)
print"---Beginning Run---"

#set rate
rate = rospy.Rate(5)
lost = False
wall_Found = False
start_time = rospy.Time.now()

#Random roaming period
print"---Finding Wall---"
while not wall_Found:
    if checkPox(1):
        wall_Found = True
        stop()
        start_time = rospy.Time.now()
    else:
        lin = 0.2
        ang = np.pi / (random.randint(10, 20))
        go(lin, ang)
        if rospy.Time.now() - start_time > rospy.Duration(secs=20):
            lost = True
    rate.sleep()

if not lost:
    print"---Wall Found---"
    lin = 0.1
    ranges = clean(0, 5, ranges)
    hold = getAlignment()
    side = 0
    if hold[0] >= abs(hold[1]):
        side = 1
    turning_Temp = (hold[side] + correctPox(1, side= side))/2.5
    go(lin, turning_Temp)

    while not rospy.is_shutdown():
        ranges = clean(0, 5, ranges)
        hold = getAlignment()
        side = 0
        if hold[0] >= abs(hold[1]):
            side = 1
        turning_Temp = (hold[side])/2.5
        go(lin, turning_Temp)
        rate.sleep()


else:
    stop()
    print("---The Robot could not find a wall!")
    