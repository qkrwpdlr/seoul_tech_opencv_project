import pyrealsense2 as rs
import numpy as np
import cv2
import math

OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "boosting": cv2.TrackerBoosting_create,
    "mil": cv2.TrackerMIL_create,
    "tld": cv2.TrackerTLD_create,
    "medianflow": cv2.TrackerMedianFlow_create,
    "mosse": cv2.TrackerMOSSE_create
}

def rsInit():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline

def select_ROI(color_image):
    cv2.namedWindow('select ROI', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('select ROI', color_image)
    cv2.putText(color_image ,"select ROI", (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
    rect = cv2.selectROI("select ROI",color_image)
    cv2.destroyWindow("select ROI")
    tracker = OPENCV_OBJECT_TRACKERS['mil']()
    tracker.init(color_image,rect)
    return tracker

def tracking(color_image,depth_frame,tracker):
    _, box = tracker.update(color_image)
    left,top,w,h = [int(v) for v in box]
    right = left + w
    bottom = top + h
    x_middle = (left + right) // 2
    y_middle = (top + bottom) // 2
    cv2.rectangle(color_image,(left,top),(right,bottom),(255,255,255),3)
    depth = depth_frame.get_distance(x_middle,y_middle)
    return color_image,depth

points = []
def click(event,x,y,flages,param):
    global points
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print(points)
        if len(points) == 2:
            dist = distance(points[0],points[1],x,y)
            print(dist)
        points = [x,y]

def distance(x1,y1,x2,y2):
    return ((x1-x2) ** 2 + (y1- y2) ** 2) ** 0.5