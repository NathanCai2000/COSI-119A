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
def checkPox(margin, cone=18):
    retu = False
    for x in range(0, cone):
        if ranges[x] <= margin:
            retu = True
    for x in range(360 - cone, 360):
        if ranges[x] <= margin:
            retu = True
    return retu

def get_ForwardPox(cone=18):
    return checkAng(360-cone, cone)

def correctPox(data, margin=1):
    retu = 0.2
    smallest = 1
    hold = []
    for a in data:
        hold + inf_Clean(a)
    return retu/min(hold)

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

def getAlignment(data):
    retu = []
    for x in range(234, 342, 36):
        if math.isinf(data[x]) and math.isinf(data[x + 36]):
            retu.append(float('inf'))
        else:
            retu.append(checkAng(x, x+36))

    if math.isinf(data[342]) and math.isinf(data[18]):
        retu.append(float('inf'))
    else:
        retu.append(checkAng(342, 18))

    for x in range(19, 127, 36):
        if math.isinf(data[x]) and math.isinf(data[x + 36]):
            retu.append(float('inf'))
        else:
            retu.append(checkAng(x, x+36))
    return retu

def clean(minV, maxV, data):
    retu = data
    for x in range(len(retu)):
        if retu[x] <= 2*maxV:
            retu[x] = min(max(minV, retu[x]), maxV)
        else:
            retu[x] = float('inf')
    return retu
    
def split(data):
    retu  = []
    for x in range(234, 342, 36):
        retu.append(data[x:x+36])
    retu.append(data[342:360] + data[0:18])
    for x in range(19, 127, 36):
        retu.append(data[x:x+36])
    return retu

def inf_Clean(data):
    retu = []
    for x in data:
        if math.isinf(x):
            retu.append(x)
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
        ang = np.pi / (random.randint(1, 10)) * random.randint(-1,1)
        go(lin, ang)
        if rospy.Time.now() - start_time > rospy.Duration(secs=20):
            lost = True
    rate.sleep()

if not lost:
    print"---Wall Found---"

    while not rospy.is_shutdown():
        if checkPox(1):
            ang 
    """
    while not rospy.is_shutdown():
        new_ranges = clean(0.2, 1, ranges)
        new_angles = getAlignment(new_ranges)
        print new_angles
        clean_new_angles = inf_Clean(new_angles)
        print clean_new_angles
        ang = sum(clean_new_angles)/ len(clean_new_angles) * correctPox(new_ranges)
        go(0.2, ang)
        rate.sleep()
        
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
        turning_Temp = (hold[side] + correctPox(1, side= side))/2.5
        go(lin, turning_Temp)
        rate.sleep()
    """
else:
    stop()
    print("---The Robot could not find a wall!---")
    