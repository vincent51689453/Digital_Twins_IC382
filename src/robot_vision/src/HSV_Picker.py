import cv2
import numpy as np

hsv_image = None
pixel = (20,60,80)   #Arbitary Value [H,S,V]  H(Hue) = 0-179, S(Saturation) = 0-255, V(Brightness) = 0-255


def color_info(event,x,y,flag,param):
    if event == cv2.EVENT_LBUTTONDOWN:  #When Left is clicked in the mouse
        pixel = hsv_image[y,x]
        upper_limit = np.array([pixel[0] + 10, pixel[1] + 10, pixel[2] + 40])
        lower_limit = np.array([pixel[0] - 10, pixel[1] - 10, pixel[2] - 40])
        print("HSV Pixel:",pixel)
        print("Lower_Limit: ",lower_limit)
        print("Upper_Limit: ",upper_limit)
        print('\n')
        #Display the masking result of the threshold value
        image_mask = cv2.inRange(hsv_image,lower_limit,upper_limit)
        cv2.imshow("Mask",image_mask)


def main():
    global hsv_image,pixel

    #Raw Image Input
    raw_image = cv2.imread("/home/vincent/vincent_dev/gazebo_ws/src/robot_vision/src/buffer.jpg")
    if raw_image is None:
        print("Image input failed!")
    cv2.imshow("Raw Image",raw_image)

    #Covert Image to HSV and create a new window to receive Mouse Click on a picture
    hsv_image = cv2.cvtColor(raw_image,cv2.COLOR_BGR2HSV)
    cv2.namedWindow('HSV_Picker')                  #A new window call HSV_Picker
    cv2.imshow("HSV_Picker",hsv_image)             #Show image on it
    cv2.setMouseCallback('HSV_Picker',color_info)  #Receive mouse click on HSV_Picker

    #Wiat until any key is pressed
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':  
    main()                  
