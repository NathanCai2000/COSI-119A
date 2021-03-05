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

#A function that splits the array into 7 directional parts
def segment(max_range=5):
    temp = []
    for x in range(234, 342, 36):
        temp.append(min(ranges[x:x+36]))
    hold = []
    for x in range(342, 360):
        hold.append(ranges[x])
    for x in range(0, 19):
        hold.append(ranges[x])
    temp.append(min(hold))
    for x in range(18, 126, 36):
        temp.append(min(ranges[x:x+36]))
    for x in range(0, len(temp)):
        if temp[x] > 5:
            temp[x] = float('inf')
    return temp

#A function similar to segment but for angles instead of ranges
def segment_Angles(max_range=5):
    temp = []
    stor1 = []
    for x in range(234, 342, 2):
        stor1.append(checkAng(x, x+1))
    temp.append(min(stor1))
    hold = []
    for x in range(342, 360, 2):
        hold.append(checkAng(x, x+1))
    for x in range(0, 19, 2):
        hold.append(checkAng(x, x+1))
    temp.append(min(hold))
    stor2 = []
    for x in range(18, 126, 2):
        stor2.append(checkAng(x, x+1))
    temp.append(min(stor2))
    return temp

# 0=no corners, 1=inner corner, 2=outer corner, 3=no wall
def check_corner(data):
    corner = 0
    inner = True
    for x in data[2:5]:
        if math.isinf(x):
            inner = False
    outer = not math.isinf(data[0]) or not math.isinf(data[6])
    for x in data[1: 6]:
        if not math.isinf(x):
            outer = False
    no_Wall = True
    for x in data:
        if not math.isinf(x):
            no_Wall = False
    if no_Wall:
        return 3
    elif outer:
        return 2
    elif inner:
        return 1
    else:
        return 0

#A function to tell if the bot has found a wall
def found_Wall(data):
    retu = False
    for x in data:
        if not math.isinf(x) and x <= 1:
            retu = True
    return retu

#Simple function for error calculations            
def getErr(margin, data):
    temp = []
    for x in data:
        if not math.isinf(x):
            temp.append(x)
    return margin - min(temp)

#Simple function to find the angle to align with wall
def get_Alignment(data):
    lowest = 0
    for x in range(1,len(data)):
        if data[lowest] > data[x] and not math.isinf(data[x]):
            lowest = x
    if lowest < 3:
        return [data[lowest], 1]
    else:
        return [data[lowest], -1]

#init node
rospy.init_node('dancer')

#subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

#calls and connects all of the publishers and subscribers
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

rospy.sleep(1)
print"---Beginning Run---"

#set rate and initial calls
rate = rospy.Rate(20)
start_time = rospy.Time.now()
max_vel = 0.3
follow_Distance = 1
last_ranges = segment()
last_err = getErr(1, last_ranges)
last_time = rospy.Time.now().to_sec()
constants = [0.1, 0.4, 0.4]

curr_Time = rospy.Time.now().to_sec()

#Random roaming loop
while not rospy.is_shutdown():
    first_Ranges = segment()
    if found_Wall(first_Ranges):
        break
    lin = min(max(random.uniform(0, .2), 0), .2)
    ang = min(max(random.uniform(-.1, .1), -.1), .1)
    go(lin, ang)

#Reset before follow
stop()
rospy.sleep(1)

#Follows the Wall
while not rospy.is_shutdown():
    
    clean_Ranges = segment()
    
    Range_err = getErr(1, clean_Ranges)
    state = check_corner(clean_Ranges)
    lin = 0
    ang = 0

    clean_Angles = segment_Angles()

    #Variable speeds for different situations
    if min(clean_Ranges) * 2 <= follow_Distance:
        lin = 0.15
    elif min(clean_Ranges) > 1.75:
        lin = 0.12
    else:
        lin = 0.1
    curr_Time = rospy.Time.now().to_sec()
    last_time = curr_Time
    #Calculating PID
    Pd = constants[0]* Range_err + constants[1]* ((Range_err - last_err)/ 0.5)
    determine = get_Alignment(clean_Ranges)
    angle_err = (determine[0] - np.pi/2 * determine[1]) * constants[2]
    ang = Pd + angle_err

    last_ranges = clean_Ranges
    
    go(lin, ang)
    rate.sleep()
