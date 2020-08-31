#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

MSG_PATH = '/home/vincent/vincent_dev/gazebo_ws/src/robot_control/src/ros_msg_bridge.txt'
MSG_debug = False

#yaw=-1.57303

#Lane distance
distance = 0
#Traffic light
left = False
#Stairs
stairs = False
#Turning position
turn = False
#ON_AREA
on_area = 0
on_area_thresh = 9000

basic_velocity_x = -0.05
basic_angular_z = 0.01

log_anchor = False
ref_alpha = 0

ROBOT_STOP = False
ROBOT_TURN = False
ROBOT_FINISH = False


counter = 0
counter2 = 0


kp,ki,kd = 2,0.1,0.8
previous_error = 0
derivative = 0
integral = 0
integral_thresh = 20
error_control = 0


def pid_controller(kp_x,ki_x,kd_x,input_dist):
    adjust = 0
    global error_control
    error_control = (input_dist - ref_alpha)/100
    #I controller
    global integral,integral_thresh
    integral = integral + error_control
    if((error_control==0)or(abs(error_control)>integral_thresh)):
        integral = 0
    #D controller
    global derivative,previous_error
    derivative = error_control - previous_error
    previous_error = error_control

    #PID controller
    pid_output = kp*error_control + ki*integral + kd*derivative
    pid_output = pid_output/10
    print("PID Controller:",pid_output)
    global basic_angular_z
    if(error_control>0):
        #need right adjust
        basic_angular_z = basic_angular_z - basic_angular_z*pid_output
    else:
        #need left adjst
        basic_angular_z = basic_angular_z + basic_angular_z*pid_output
    if(basic_angular_z>=10)or(basic_angular_z<=-10):
        basic_angular_z = 0.01
    print("Angular z=",basic_angular_z)
    return basic_angular_z

def ros_setup():
    #starts a new node
    rospy.init_node('robot', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    return vel_msg,velocity_publisher
                 
def main():
    speed_msg,ros_publisher = ros_setup()
    print("ROS node initialization is done!")
    while not rospy.is_shutdown():
        global distance,on_area,stairs,left
        #Receive vision message from AI
        msg_file = open(MSG_PATH,'r')
        vision_msg = msg_file.readlines()
        #print(len(vision_msg[0]))
        if(len(vision_msg)==1):
            #Lane distanc
            distance = int(vision_msg[0][1])*100+int(vision_msg[0][2])*10+int(vision_msg[0][3])*1
            #ON_AREA
            on_area = int(vision_msg[0][8])*1000+int(vision_msg[0][9])*100+int(vision_msg[0][10])*10+int(vision_msg[0][11])
            #Traffic light
            if(vision_msg[0][4]=='R'):
                left = False
            else:
                left = True
            #Stairs
            if(vision_msg[0][6]=='X'):
                stairs = False
            else:
                stairs = True
            if(MSG_debug == True):
                #Raw message
                print("Message=",vision_msg[0])
                print("Distance=",distance)
                print("ON_AREA=",on_area)
                print("LEFT=",left)
                print("STAIRS=",stairs)
            global log_anchor
            if(log_anchor==False):
                #Log reference when the system begins
                global ref_alpha
                ref_alpha = distance
                log_anchor = True
            previous_msg = vision_msg

        global ROBOT_STOP,ROBOT_TURN,ROBOT_FINISH
        if((on_area<on_area_thresh)and(ROBOT_STOP==False)):
            #Should not turn left/right
            adjust = pid_controller(kp,ki,kd,distance)
            speed_msg.linear.x = basic_velocity_x
            speed_msg.linear.y = 0
            speed_msg.linear.z = 0
            speed_msg.angular.x = 0
            speed_msg.angular.y = 0
            speed_msg.angular.z = adjust
            print(speed_msg)
            ros_publisher.publish(speed_msg)
        else:
            speed_msg.linear.x = 0
            speed_msg.linear.y = 0
            speed_msg.linear.z = 0
            speed_msg.angular.x = 0
            speed_msg.angular.y = 0
            speed_msg.angular.z = 0
            #print("Ready for turning")
            ROBOT_STOP = True
            ros_publisher.publish(speed_msg)
  

        if(ROBOT_STOP == True):
            global counter
            if(counter <= 16000):
                speed_msg.linear.x = -0.01
                speed_msg.linear.y = 0
                speed_msg.linear.z = 0
                speed_msg.angular.x = 0
                speed_msg.angular.y = 0
                if(left==True):
                    print("counter:",counter)
                    speed_msg.angular.z = 1.2
                else:
                    speed_msg.angular.z = -1.2
                ros_publisher.publish(speed_msg) 
            else:
                ROBOT_TURN = True
            counter += 1  


        if(ROBOT_TURN == True):
            global counter2
            if(counter2 <= 9500):
                speed_msg.linear.x = basic_velocity_x
                speed_msg.linear.y = 0
                speed_msg.linear.z = 0
                speed_msg.angular.x = 0
                speed_msg.angular.y = 0
                speed_msg.angular.z = 0
                ros_publisher.publish(speed_msg) 
                print("counter:",counter2)
            else:
                ROBOT_FINISH = True
            counter2 += 1 


        if(ROBOT_FINISH == True):
            speed_msg.linear.x = 0
            speed_msg.linear.y = 0
            speed_msg.linear.z = 0
            speed_msg.angular.x = 0
            speed_msg.angular.y = 0
            speed_msg.angular.z = 0
            ros_publisher.publish(speed_msg)
            print("FINISHED!")  

        
            
        print("\r\n")        
    
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
