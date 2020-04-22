import pyrealsense2 as rs
import numpy as np
import cv2
from core import rsInit,select_ROI,tracking,click,select_ROI2
import math
import matplotlib.pyplot as plt

pipeline,pipe_profile  = rsInit()
size = 1

position = np.float32([[1,0,0],[0,1,0]])
try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        cv2.namedWindow("MAIN_SCREEN", cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow("MAIN_SCREEN",0,0)
        color_image = cv2.resize(color_image, None, fx=size, fy=size, interpolation=cv2.INTER_CUBIC)
        color_image = cv2.warpAffine(color_image,position,(len(color_image[0]),len(color_image)))
        def drag_mouse(event,x,y,flags,params):
            global size
            if event == cv2.EVENT_MOUSEWHEEL:
                if flags > 0:
                    size += 0.1
                else:
                    if size < 1:
                        size == 1
                    else:
                        size -= 0.1
            if event == cv2.EVENT_LBUTTONDBLCLK:
                print((x+position[0][2])/size,(y+position[1][2])/size)
        cv2.setMouseCallback("MAIN_SCREEN", drag_mouse)
        cv2.imshow("MAIN_SCREEN", color_image)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        if key == ord('w'):
            position[1][2] += 100
        if key == ord("a"):
            position[0][2] += 100
        if key == ord("s"):
            position[1][2] -= 100
        if key == ord("d"):
            position[0][2] -= 100
        
finally:
    pass
