#!/usr/bin/env python

#^^Shebang Line

#Imports
import math     #For easy radians/s calculations with accurate Pi
import rospy
from geometry_msgs.msg import Twist

#Creating the publisher for the Node
example_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#Setting the Name of the Node
rospy.init_node('movemeters')

#Method for simply controlling the bot's movement based off of speed and duration of time
#speedlin: The Linear speed for the bot (m/s)
#speedang: The angular speed of the robot (rad/s)
#time: The duration of time to go at this speed (s)
"""
Bugs and issues:
Though the code is sound at controlling the bot, it still requires a 0 speed call at the time at the end of the
initial function call
ex:
go(x,y,t)
go(0,0,t)
As the robot does not stop after the function is completed, however, it can seamlessly transition into another movement
, as this bug is only applicable to if it needs to stops. There is also potentially a slight inaccuracy with the stopping 
time as the speed outputs are published after the time check, so there may be a tiny delay in stopping.
"""
def go(speedlin, speedang, time):
    starting_time = rospy.Time.now().to_sec()                       #Stores the time in which the function was first called in seconds
    Temp = True                                                     #Boolean for controlling the while loop
    while Temp:                                                     #Beginning of while loop
        passed_time = rospy.Time.now().to_sec() - starting_time     #Gets the time passed by comparing the initial start time with the current time
        twist = Twist()                                             #Creates a Twist()
        twist.linear.x = speedlin                                   #Sets the linear speed to the given speedlin
        twist.angular.z = speedang                                  #Sets the angular speed to the given speedang
        example_pub.publish(twist)                                  #Publishes the new twist speeds
        if passed_time > time:                                      #Checks if the set duration has passed
            Temp = False                                            #If the set duration has passed, it will end the while loop and thus function

rospy.sleep(5)              #This is needed as rospy.Time.now() and rospy.get_rostime() will return the wrong times if not done once before they're called
go(0.15, 0, 10)             #Moves the bot forward in a straight line at 0.15m/s for 10s, it is at such a slow speed to minimise deviation
go(0, math.pi/10, 10)       #Rotates the bot 180 degrees for 10s, it is at such a slow speed to minimise deviation
go(0.15, 0, 10)             #Moves the bot forward in a straight line at 0.15m/s for 10s now in the opposite direction due to the turn
go(0, 0, 1)                 #A Move call to stop the bot, and it stops for 1s to be sure it is at full stop