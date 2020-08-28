#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from cv_bridge.boost.cv_bridge_boost import getCvType
from sensor_msgs.msg import Image
import math

img_height = 640
img_width = 920


ref_x = img_width/2
ref_y = img_height/2+200

def region_of_interest(img,vertices):
    mask = np.zeros_like(img)
    #Masking color
    match_mask_color = 255
    #Fill inside the polygon
    cv2.fillPoly(mask,vertices,match_mask_color)
    masked_image = cv2.bitwise_and(img,mask)
    return masked_image

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #use $rostopic list to find out the correct topic about images
    self.image_sub = rospy.Subscriber('/robot/camera1/image_raw', 
                                      Image, self.image_callback)
  def image_callback(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg)
    #Convert BRG image to 
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    #cv2.imshow("camera_view(raw)", image )
    
    min_x = 500
    min_distance = 1000
    distance = 0
    y_thresh = 50
    
    #Lane Detection
    region_of_interest_vertices = [(0,(img_height-50)),(img_width/2,img_height/2),(img_width,(img_height-50))]
    lane_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(lane_img,50,200)
    cropped_image = region_of_interest(edges,np.array([region_of_interest_vertices],np.int32))
    #cropped_image = cv2.resize(cropped_image, (640, 420))
    #cv2.imshow("ROI", cropped_image )
    lines = cv2.HoughLinesP(cropped_image,1,np.pi/180,100,minLineLength=10,maxLineGap=250)
    #Reference
    cv2.circle(image,(ref_x,ref_y),4,(0,255,0),-1)
    cv2.putText(image,"REFERENCE",(ref_x,ref_y-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
    if lines is not None:
        for line in lines:
            x1,y1,x2,y2 = line[0]
            mid_x = int((x1+x2)/2)
            mid_y = int((y1+y2)/2)
            distance = int(math.sqrt((mid_x-ref_x)**2+(mid_y-ref_y)**2))
            #ignore horizontal line
            if(abs(y2-y1) > y_thresh):
                print("x1={} y1={} x2={} y2={} | Distance={}".format(x1,y1,x2,y2,distance))
                #Find the shortest distance
                if((mid_x<=ref_x)and(min_distance>distance)):
                    min_x = mid_x
                    min_distance=distance

                cv2.line(image,(x1,y1),(x2,y2),(255,0,0),3)
                cv2.circle(image,(mid_x,mid_y),4,(0,0,255),-1)
            
    image = cv2.resize(image, (640, 420))
    cv2.imshow("lane_detector", image )
    # END BRIDGE

    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
