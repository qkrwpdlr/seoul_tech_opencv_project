import pyrealsense2 as rs
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "boosting": cv2.TrackerBoosting_create,
    "mil": cv2.TrackerMIL_create,
    "tld": cv2.TrackerTLD_create,
    "medianflow": cv2.TrackerMedianFlow_create,
    "mosse": cv2.TrackerMOSSE_create
}
depth_min = 0.01
depth_max = 2
class CV:
    def __init__(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipe_profile = pipeline.start(config)
        self.pipeline = pipeline
        self.pipe_profile = pipe_profile
        self.frames = pipeline.wait_for_frames()
    def _get_frame(self):
        self.depth_frame = self.frames.get_depth_frame()
        self.color_frame = self.frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())  

    def select_ROI(self):
        self._get_frame()
        cv2.imshow('select ROI', self.color_image)
        cv2.putText(self.color_image ,"select ROI", (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
        self.rect = cv2.selectROI("select ROI",self.color_image)
        cv2.destroyWindow("select ROI")
        self.tracker = OPENCV_OBJECT_TRACKERS['csrt']()
        self.tracker.init(self.color_image,self.rect)
    def _color_px_to_depth_px(self,x,y):
        depth_scale = self.pipe_profile.get_device().first_depth_sensor().get_depth_scale()
        depth_intrin = self.pipe_profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_intrin = self.pipe_profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        depth_to_color_extrin =  self.pipe_profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(self.pipe_profile.get_stream(rs.stream.color))
        color_to_depth_extrin =  self.pipe_profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(self.pipe_profile.get_stream(rs.stream.depth))
        depth_point = rs.rs2_project_color_pixel_to_depth_pixel(self.depth_frame.get_data(), depth_scale, depth_min, depth_max, depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x_middle,y_middle])
        return depth_point
    def tracker(self):
        _, box = self.tracker.update(self.color_image)
        left,top,w,h = [int(v) for v in box]
        right = left + w
        bottom = top + h
        x_middle = (left + right) // 2
        y_middle = (top + bottom) // 2
        depth_point = self._color_px_to_depth_px(x_middle,y_middle)
        cv2.rectangle(self.color_image,(left,top),(right,bottom),(255,255,255),3)
        depth = self.depth_frame.get_distance(int(depth_point[0]),int(depth_point[1]))
        return depth

    def _click(self):
        pass
        # if event == cv2.EVENT_LBUTTONDBLCLK:
        #     depth_frame = self.depth_frame
        #     color_frame = self.color_frame
        #     color_image = self.color_image
        #     depth = depth_frame.get_distance(x1,y1)
        #     print("depth : ",depth)
        #     cv2.circle(color_image,(x1,y1), 100,(255,0,0),-1)
        #     if len(point) == 2:
        #         x2 = point[0]
        #         y2 = point[1]
        #         depth = depth_frame.get_distance((x1+x2)//2,(y1+y2)//2)
        #         fx,fy = rs.rs2_fov(color_frame.profile.as_video_stream_profile().intrinsics)
        #         realWdith = (abs(x1-x2) ) * 2 * math.tan(math.radians(fx / 2)) * depth / color_frame.width
        #         realHeight = (abs(y1-y2) )  * 2 * math.tan(math.radians(fy / 2)) * depth / color_frame.width
        #     point = [x1,y1]
