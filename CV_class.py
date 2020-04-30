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
        self.size = 1

    def _get_frame(self):
        self.frames = self.pipeline.wait_for_frames()
        self.depth_frame = self.frames.get_depth_frame()
        self.color_frame = self.frames.get_color_frame()
        if not self.depth_frame or not self.color_frame:
            return
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())  

    def set_size(self,size):
        self.size = size

    def select_ROI(self):
        self._get_frame()
        size_screen_name = 'resize'
        screen_name = 'select ROI'
        position = np.float32([[1,0,0],[0,1,0]])
        while True:
            cv2.setMouseCallback(size_screen_name, _drag_mouse,self)
            cv2.namedWindow(size_screen_name, cv2.WINDOW_FULLSCREEN)
            cv2.moveWindow(size_screen_name,0,0)
            color_image = cv2.resize(self.color_image, None, fx=self.size, fy=self.size, interpolation=cv2.INTER_CUBIC)
            color_image = cv2.warpAffine(color_image,position,(len(color_image[0]),len(color_image)))
            cv2.putText(color_image ,"resize and key Q", (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
            cv2.imshow(size_screen_name, color_image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            if key == ord('w'):
                position[1][2] += 100
            if key == ord("a"):
                position[0][2] += 100
            if key == ord("s"):
                position[1][2] -= 100
            if key == ord("d"):
                position[0][2] -= 100
        box = cv2.selectROI(screen_name,color_image)
        left,top,w,h =  [int(v) for v in box]
        left -= position[0][2]
        top -= position[1][2]
        left //= self.size
        top //= self.size
        w //= self.size
        h //= self.size
        self.origin_rect = (int(left),int(top),int(w),int(h))
        cv2.destroyWindow(screen_name)
        cv2.destroyWindow(size_screen_name)
        self.tracker = OPENCV_OBJECT_TRACKERS['csrt']()
        self.tracker.init(self.color_image,self.origin_rect)

    def _color_px_to_depth_px(self,x,y):
        depth_scale = self.pipe_profile.get_device().first_depth_sensor().get_depth_scale()
        depth_intrin = self.pipe_profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        color_intrin = self.pipe_profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        depth_to_color_extrin =  self.pipe_profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(self.pipe_profile.get_stream(rs.stream.color))
        color_to_depth_extrin =  self.pipe_profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(self.pipe_profile.get_stream(rs.stream.depth))
        depth_point = rs.rs2_project_color_pixel_to_depth_pixel(self.depth_frame.get_data(), depth_scale, depth_min, depth_max, depth_intrin, color_intrin, depth_to_color_extrin, color_to_depth_extrin, [x,y])
        return depth_point

    def tracking(self):
        _, box = self.tracker.update(self.color_image)
        left,top,w,h = [int(v) for v in box]
        right = left + w
        bottom = top + h
        x_middle = (left + right) // 2
        y_middle = (top + bottom) // 2
        depth_point = self._color_px_to_depth_px(x_middle,y_middle)
        cv2.rectangle(self.color_image,(left,top),(right,bottom),(255,255,255),3)
        depth = self.depth_frame.get_distance(int(depth_point[0]),int(depth_point[1]))
        return depth,box

    def get_fov(self):
        return rs.rs2_fov(self.color_frame.profile.as_video_stream_profile().intrinsics)

    def get_color_frame(self):
        return self.color_frame
    # def get_real_x_size(self,x_size):
    #     a = origin_point[0]
    #     a += origin_point[2] / 2
    #     b = box[0]
    #     b += box[2] / 2
    #     fx,fy = rs.rs2_fov(color_frame.profile.as_video_stream_profile().intrinsics)
    #     realWdith = (abs(a-b)) * 2 * math.tan(math.radians(fx / 2)) * depth / color_frame.width
    #     return realWdith
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

    def show_img(self):
        cv2.imshow("MAIN_SCREEN",self.color_image)

    def write_text(self,text):
        cv2.putText(self.color_image ,text, (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
def _drag_mouse(event,x,y,flags,params):
    if event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0:
            params.set_size(params.size + 0.1)
        else:
            if params.size < 1:
                params.set_size(1)
            else:
                params.set_size(params.size - 0.1)
