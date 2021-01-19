"""visualization.py

The BBoxVisualization class implements drawing of nice looking
bounding boxes based on object detection results.
"""


import numpy as np
import cv2


# Constants
ALPHA = 0.5
FONT = cv2.FONT_HERSHEY_PLAIN
TEXT_SCALE = 1.0
TEXT_THICKNESS = 1
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


def gen_colors(num_colors):
    """Generate different colors.

    # Arguments
      num_colors: total number of colors/classes.

    # Output
      bgrs: a list of (B, G, R) tuples which correspond to each of
            the colors/classes.
    """
    import random
    import colorsys

    hsvs = [[float(x) / num_colors, 1., 0.7] for x in range(num_colors)]
    random.seed(1234)
    random.shuffle(hsvs)
    rgbs = list(map(lambda x: list(colorsys.hsv_to_rgb(*x)), hsvs))
    bgrs = [(int(rgb[2] * 255), int(rgb[1] * 255),  int(rgb[0] * 255))
            for rgb in rgbs]
    return bgrs


def draw_boxed_text(img, text, topleft, color):
    """Draw a transluent boxed text in white, overlayed on top of a
    colored patch surrounded by a black border. FONT, TEXT_SCALE,
    TEXT_THICKNESS and ALPHA values are constants (fixed) as defined
    on top.

    # Arguments
      img: the input image as a numpy array.
      text: the text to be drawn.
      topleft: XY coordinate of the topleft corner of the boxed text.
      color: color of the patch, i.e. background of the text.

    # Output
      img: note the original image is modified inplace.
    """
    assert img.dtype == np.uint8
    img_h, img_w, _ = img.shape
    if topleft[0] >= img_w or topleft[1] >= img_h:
        return
    margin = 3
    size = cv2.getTextSize(text, FONT, TEXT_SCALE, TEXT_THICKNESS)
    w = size[0][0] + margin * 2
    h = size[0][1] + margin * 2
    # the patch is used to draw boxed text
    patch = np.zeros((h, w, 3), dtype=np.uint8)
    patch[...] = color
    cv2.putText(patch, text, (margin+1, h-margin-2), FONT, TEXT_SCALE,
                WHITE, thickness=TEXT_THICKNESS, lineType=cv2.LINE_8)
    cv2.rectangle(patch, (0, 0), (w-1, h-1), BLACK, thickness=1)
    w = min(w, img_w - topleft[0])  # clip overlay at image boundary
    h = min(h, img_h - topleft[1])
    # Overlay the boxed text onto region of interest (roi) in img
    roi = img[topleft[1]:topleft[1]+h, topleft[0]:topleft[0]+w, :]
    cv2.addWeighted(patch[0:h, 0:w, :], ALPHA, roi, 1 - ALPHA, 0, roi)
    return img


class BBoxVisualization():
    """BBoxVisualization class implements nice drawing of boudning boxes.

    # Arguments
      cls_dict: a dictionary used to translate class id to its name.
    """

    def __init__(self, cls_dict):
        self.cls_dict = cls_dict
        self.colors = gen_colors(len(cls_dict))

    def draw_bboxes(self, img, box, conf, cls):
        """Draw detected bounding boxes on the original image."""
        x_center,y_center = 0,0
        green_on_y,green_off_y = 0,0

        ON = False
        OFF = False
        STAIRS = False
        MSG = 'A'
        msg_area = 'A'

        ON_AREA = 0

        for bb, cf, cl in zip(box, conf, cls):
            cl = int(cl)
            if(cf>=0.85):
                y_min, x_min, y_max, x_max = bb[0], bb[1], bb[2], bb[3]
                x_center = (x_min+x_max)/2
                y_center = (y_min+y_max)/2
                color = self.colors[cl]
                cv2.rectangle(img, (x_min, y_min), (x_max, y_max), color, 2)
                txt_loc = (max(x_min+2, 0), max(y_min+2, 0))
                cls_name = self.cls_dict.get(cl, 'CLS{}'.format(cl))
                txt = '{} {:.2f}'.format(cls_name, cf)
                img = draw_boxed_text(img, txt, txt_loc, color)
                if(cl == 1):
                    green_on_x = x_center
                    ON = True
                    ON_AREA = (x_max-x_min)*(y_max-y_min)
                    #print("area:",ON_AREA)
                if(cl == 2):
                    green_off_x = x_center
                    OFF = True
                if(cl == 3):
                    STAIRS = True

        msg_length = len(str(ON_AREA))
        if(ON_AREA==0):
            msg_area = "0000"
        elif(msg_length==1):
            msg_area = "000" + str(ON_AREA)
        elif(msg_length==2):
            msg_area = "00" + str(ON_AREA)
        elif(msg_length==3):
            msg_area = "0" + str(ON_AREA)
        elif(msg_length==4):
            msg_area = str(ON_AREA)
        else:
            msg_area = "9999"

        if(ON and OFF):
            if(green_on_x<green_off_x):
              #turn left
              MSG = 'L'
            else:
              #turn right
              MSG = 'R'
        if(STAIRS):
            MSG += '_S'
        else:
            MSG += '_X'

        MSG += '_'+msg_area

        return img,MSG
