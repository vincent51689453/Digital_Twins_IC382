#!/usr/bin/env python3
import sys
import time
import logging
import argparse

import numpy as np
import cv2
import tensorflow as tf
import tensorflow.contrib.tensorrt as trt

import math

from utils.camera import add_camera_args, Camera
from utils.od_utils import read_label_map, build_trt_pb, load_trt_pb, \
                           write_graph_tensorboard, detect
from utils.visualization import BBoxVisualization

img_height = 640
img_width = 920


ref_x = int(img_width/2)
ref_y = int(img_height/2+200)

# Constants
DEFAULT_MODEL = 'ssd_inception_v2_coco'
DEFAULT_LABELMAP = 'third_party/models/research/object_detection/' \
                   'data/mscoco_label_map.pbtxt'
WINDOW_NAME = 'CameraTFTRTDemo'
BBOX_COLOR = (0, 255, 0)  # green


def region_of_interest(img,vertices):
    mask = np.zeros_like(img)
    #Masking color
    match_mask_color = 255
    #Fill inside the polygon
    cv2.fillPoly(mask,vertices,match_mask_color)
    masked_image = cv2.bitwise_and(img,mask)
    return masked_image

def parse_args():
    """Parse input arguments."""
    desc = ('This script captures and displays live camera video, '
            'and does real-time object detection with TF-TRT model '
            'on Jetson TX2/TX1/Nano')
    parser = argparse.ArgumentParser(description=desc)
    parser = add_camera_args(parser)
    parser.add_argument('--model', dest='model',
                        help='tf-trt object detecion model '
                        '[{}]'.format(DEFAULT_MODEL),
                        default=DEFAULT_MODEL, type=str)
    parser.add_argument('--build', dest='do_build',
                        help='re-build TRT pb file (instead of using'
                        'the previously built version)',
                        action='store_true')
    parser.add_argument('--tensorboard', dest='do_tensorboard',
                        help='write optimized graph summary to TensorBoard',
                        action='store_true')
    parser.add_argument('--labelmap', dest='labelmap_file',
                        help='[{}]'.format(DEFAULT_LABELMAP),
                        default=DEFAULT_LABELMAP, type=str)
    parser.add_argument('--num-classes', dest='num_classes',
                        help='(deprecated and not used) number of object '
                        'classes', type=int)
    parser.add_argument('--confidence', dest='conf_th',
                        help='confidence threshold [0.3]',
                        default=0.3, type=float)
    args = parser.parse_args()
    return args


def open_display_window(width, height):
    """Open the cv2 window for displaying images with bounding boxeses."""
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, width, height)
    cv2.moveWindow(WINDOW_NAME, 0, 0)
    cv2.setWindowTitle(WINDOW_NAME, 'AI Analyzer')


def draw_help_and_fps(img, fps):
    """Draw help message and fps number at top-left corner of the image."""
    help_text = "'Esc' to Quit, 'H' for FPS & Help, 'F' for Fullscreen"
    font = cv2.FONT_HERSHEY_PLAIN
    line = cv2.LINE_AA

    fps_text = 'FPS: {:.1f}'.format(fps)
    cv2.putText(img, help_text, (11, 20), font, 1.0, (32, 32, 32), 4, line)
    cv2.putText(img, help_text, (10, 20), font, 1.0, (240, 240, 240), 1, line)
    cv2.putText(img, fps_text, (11, 50), font, 1.0, (32, 32, 32), 4, line)
    cv2.putText(img, fps_text, (10, 50), font, 1.0, (240, 240, 240), 1, line)
    return img


def set_full_screen(full_scrn):
    """Set display window to full screen or not."""
    prop = cv2.WINDOW_FULLSCREEN if full_scrn else cv2.WINDOW_NORMAL
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, prop)


def loop_and_detect(cam, tf_sess, conf_th, vis, od_type):
    """Loop, grab images from camera, and do object detection.

    # Arguments
      cam: the camera object (video source).
      tf_sess: TensorFlow/TensorRT session to run SSD object detection.
      conf_th: confidence/score threshold for object detection.
      vis: for visualization.
    """
    show_fps = True
    full_scrn = False
    fps = 0.0
    tic = time.time()
    distance = 0
    mid_x,mid_y = 0,0
    y_thresh = 50   
    while True:
        min_x = 500
        min_distance = 1000 
        img = cv2.imread('/home/vincent/vincent_dev/gazebo_ws/src/robot_vision/src/buffer.jpg')
        
        if cv2.getWindowProperty(WINDOW_NAME, 0) < 0:
            # Check to see if the user has closed the display window.
            # If yes, terminate the while loop.
            break

        #img = cam.read()
        if img is not None:
            #Lane Detection
            #Verticeis of cropped polygon
            region_of_interest_vertices = [(0,(img_height-50)),(img_width/2,img_height/2),(img_width,(img_height-50))]
            lane_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(lane_img,50,200)
            cropped_image = region_of_interest(edges,np.array([region_of_interest_vertices],np.int32))
            #cv2.imshow('cropped',cropped_image)
            lines = cv2.HoughLinesP(cropped_image,1,np.pi/180,100,minLineLength=10,maxLineGap=250)
            #Reference
            cv2.circle(img,(ref_x,ref_y),4,(0,255,0),-1)
            cv2.putText(img,"REFERENCE",(ref_x,ref_y-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
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

                        cv2.line(img,(x1,y1),(x2,y2),(255,255,0),3)
                        cv2.circle(img,(mid_x,mid_y),4,(255,0,0),-1)  
                print("Min X={} | Min_distance={}".format(min_x,min_distance))
         
            #Lane Detection finished
            box, conf, cls = detect(img, tf_sess, conf_th, od_type=od_type)
            img = vis.draw_bboxes(img, box, conf, cls)
            if show_fps:
                img = draw_help_and_fps(img, fps)
                
                
                
                
                
            img = cv2.resize(img, (640, 420))   
            cv2.imshow(WINDOW_NAME, img)
            toc = time.time()
            curr_fps = 1.0 / (toc - tic)
            # calculate an exponentially decaying average of fps number
            fps = curr_fps if fps == 0.0 else (fps*0.9 + curr_fps*0.1)
            tic = toc

        key = cv2.waitKey(1)
        if key == 27:  # ESC key: quit program
            break
        elif key == ord('H') or key == ord('h'):  # Toggle help/fps
            show_fps = not show_fps
        elif key == ord('F') or key == ord('f'):  # Toggle fullscreen
            full_scrn = not full_scrn
            set_full_screen(full_scrn)


def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)
    # Ask tensorflow logger not to propagate logs to parent (which causes
    # duplicated logging)
    logging.getLogger('tensorflow').propagate = False

    args = parse_args()
    logger.info('called with args: %s' % args)

    # build the class (index/name) dictionary from labelmap file
    logger.info('reading label map')
    cls_dict = read_label_map(args.labelmap_file)

    pb_path = './data/{}_trt.pb'.format(args.model)
    log_path = './logs/{}_trt'.format(args.model)
    if args.do_build:
        logger.info('building TRT graph and saving to pb: %s' % pb_path)
        build_trt_pb(args.model, pb_path)

    logger.info('opening camera device/file')
    cam = Camera(args)
    cam.open()
    if not cam.is_opened:
        sys.exit('Failed to open camera!')

    logger.info('loading TRT graph from pb: %s' % pb_path)
    trt_graph = load_trt_pb(pb_path)

    logger.info('starting up TensorFlow session')
    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True
    #tf_sess = tf.Session(config=tf_config, graph=trt_graph)
    tf_sess = tf.Session(config=tf.ConfigProto(allow_soft_placement=True,log_device_placement=True),graph=trt_graph)

    if args.do_tensorboard:
        logger.info('writing graph summary to TensorBoard')
        write_graph_tensorboard(tf_sess, log_path)

    logger.info('warming up the TRT graph with a dummy image')
    od_type = 'faster_rcnn' if 'faster_rcnn' in args.model else 'ssd'
    dummy_img = np.zeros((720, 1280, 3), dtype=np.uint8)
    _, _, _ = detect(dummy_img, tf_sess, conf_th=.3, od_type=od_type)

    cam.start()  # ask the camera to start grabbing images

    # grab image and do object detection (until stopped by user)
    logger.info('starting to loop and detect')
    vis = BBoxVisualization(cls_dict)
    open_display_window(cam.img_width, cam.img_height)
    loop_and_detect(cam, tf_sess, args.conf_th, vis, od_type=od_type)

    logger.info('cleaning up')
    cam.stop()  # terminate the sub-thread in camera
    tf_sess.close()
    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
