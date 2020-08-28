#!/usr/bin/env python


# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from cv_bridge.boost.cv_bridge_boost import getCvType
from sensor_msgs.msg import Image


FILE_PATH = '/home/vincent/vincent_dev/gazebo_ws/src/robot_vision/src/frame_buffer/'
IMAGE_HEAD = 'frame_'
IMAGE_FORMAT = '.jpg'
IMAGE_ID = 0

frame_counter = 0
interval = 2

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #use $rostopic list to find out the correct topic about images
    self.image_sub = rospy.Subscriber('/robot/camera1/image_raw', 
                                      Image, self.image_callback)
  def image_callback(self, msg):
    # BEGIN BRIDGE
    image = self.bridge.imgmsg_to_cv2(msg)
    global IMAGE_ID, frame_counter
    #Convert BRG image to 
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if(frame_counter % interval == 0):
        cv2.imshow("camera_view(raw)", image)
        IMG_SAVE = FILE_PATH + IMAGE_HEAD + str(IMAGE_ID) + IMAGE_FORMAT
        cv2.imwrite(IMG_SAVE,image)
        print("Image Saved at:"+IMG_SAVE)
        IMAGE_ID += 1
    
    frame_counter += 1
    # END BRIDGE

    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
