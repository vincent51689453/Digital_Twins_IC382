#!/usr/bin/env python3

"""
This is a very simple control script to test the robot.
q : turn the robot to left  (angular z += basic left)
e : turn the robot to right (angular z += basic right)
w : add forward power
x : reset
"""

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard


basic_velocity_x = -0.05

x_step = -0.05
left_step = 0.05
right_step = -0.05

adjust = 0
power = basic_velocity_x


def ros_setup():
    #starts a new node
    rospy.init_node('robot', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    return vel_msg,velocity_publisher
    
def on_press (key):
    global adjust,power
    try:
        if(key.char=='q'):
            #Turn Left
            adjust += left_step
        if(key.char=='e'):
            #Turn Right
            adjust += right_step
        if(key.char=='w'):
            #Forward (increase the speed)
            power += x_step
            adjust = 0
        if(key.char=='x'):
            #Stop
            power = 0
            adjust = 0         
    except AttributeError:
        print('special key {0} pressed'.format(key))

def on_release(key):
    #print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

       
        
    
   
    
def main():
    speed_msg,ros_publisher = ros_setup()
    print("ROS node initialization is done!")
    #This script supported keyboard control
    listener = keyboard.Listener(on_press=on_press,on_release=on_release)
    listener.start()
    while not rospy.is_shutdown():
        speed_msg.linear.x = power
        speed_msg.linear.y = 0
        speed_msg.linear.z = 0
        speed_msg.angular.x = 0
        speed_msg.angular.y = 0
        global adjust
        speed_msg.angular.z = adjust
        print(speed_msg)
        ros_publisher.publish(speed_msg)
    
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
