import pyrealsense2 as rs
import numpy as np
import cv2
import math

depth_min = 0.11 #meter
depth_max = 1.0 #meter

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
    pipe_profile = pipeline.start(config)
    return pipeline , pipe_profile 

def select_ROI(color_image):
    cv2.namedWindow('select ROI', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('select ROI', color_image)
    cv2.putText(color_image ,"select ROI", (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
    rect = cv2.selectROI("select ROI",color_image)
    cv2.destroyWindow("select ROI")
    tracker = OPENCV_OBJECT_TRACKERS['csrt']()
    tracker.init(color_image,rect)
    return tracker,rect
def select_ROI2(color_image):
    cv2.namedWindow('select ROI', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('select ROI', color_image)
    cv2.putText(color_image ,"select ROI", (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
    rect = cv2.selectROI("select ROI",color_image)
    cv2.destroyWindow("select ROI")
    tracker = OPENCV_OBJECT_TRACKERS['csrt']()
    tracker.init(color_image,rect)
    return tracker,rect

def tracking(color_image,depth_frame,tracker,depth_image,profile):
    global depth_min,depth_max
    _, box = tracker.update(color_image)
    left,top,w,h = [int(v) for v in box]
    right = left + w
    bottom = top + h
    x_middle = (left + right) // 2
    y_middle = (top + bottom) // 2

    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    depth_intrin = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    color_intrin = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    depth_to_color_extrin =  profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.color))
    color_to_depth_extrin =  profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to( profile.get_stream(rs.stream.depth))
    
    depth_point = rs.rs2_project_color_pixel_to_depth_pixel(depth_frame.get_data(), depth_scale, depth_min, depth_max,
                depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x_middle,y_middle])

    cv2.rectangle(color_image,(left,top),(right,bottom),(255,255,255),3)
    cv2.rectangle(depth_image,(left,top),(right,bottom),(255,255,255),3)
    depth = depth_frame.get_distance(int(depth_point[0]),int(depth_point[1]))
    return color_image,depth,box,depth_image

point = []
def click(event,x1,y1,flages,param):
    global point
    if event == cv2.EVENT_LBUTTONDBLCLK:
        depth_frame = param[0]
        color_frame = param[1]
        color_image = param[2]
        depth = depth_frame.get_distance(x1,y1)
        print("depth : ",depth)
        cv2.circle(color_image,(x1,y1), 100,(255,0,0),-1)
        if len(point) == 2:
            x2 = point[0]
            y2 = point[1]
            depth = depth_frame.get_distance((x1+x2)//2,(y1+y2)//2)
            fx,fy = rs.rs2_fov(color_frame.profile.as_video_stream_profile().intrinsics)
            realWdith = (abs(x1-x2) ) * 2 * math.tan(math.radians(fx / 2)) * depth / color_frame.width
            realHeight = (abs(y1-y2) )  * 2 * math.tan(math.radians(fy / 2)) * depth / color_frame.width
            print(realHeight,realWdith)
        point = [x1,y1]

def distance(x1,y1,x2,y2):
    return ((x1-x2) ** 2 + (y1- y2) ** 2) ** 0.5

def cmPerPx(depth_frame,x1,y1,x2,y2,scale):
    depth1 = depth_frame.get_distance(x1, y1)
    depth2 = depth_frame.get_distance(x2, y2)
    # fx,fy = rs.rs2_fov(depth_frame.profile.as_video_stream_profile().intrinsics)
    # print(fx,fy)
    # realWdith = (abs(x1-x2) ) * 2 * math.tan(math.radians(fx / 2)) * depth1 / depth_frame.width
    # realHeight = (abs(y1-y2) )  * 2 * math.tan(math.radians(fy / 2)) * depth1 / depth_frame.width
    # print(realWdith,realHeight,depth1 - depth2)
    intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    x = rs.rs2_deproject_pixel_to_point(intrin,[x1,y1],depth1)
    y = rs.rs2_deproject_pixel_to_point(intrin,[x2,y2],depth2)
    print("depth1 : ",depth1)
    print("depth1 : ",depth2)
    print(((x[0]-y[0])**2 +(x[1]-y[1])**2 +(x[2]-y[2])**2)**0.5 )
    # return (realWdith ** 2 + realHeight ** 2 + (depth1 - depth2)**2)**(0.5)