#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from cv_bridge.boost.cv_bridge_boost import getCvType
from sensor_msgs.msg import Image

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
    cv2.imshow("camera_view(raw)", image )
    # END BRIDGE

    cv2.waitKey(3)

rospy.init_node('follower')
print("haha")
follower = Follower()
rospy.spin()
# END ALL
