import pyrealsense2 as rs
import numpy as np
import cv2
from core import rsInit,select_ROI,tracking,click

MAIN_SCREEN = "RealSense"
pipeline = rsInit()
isTracker = False

try:
    while True:
        # 카메라 읽는 부분 START
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # 카메라 읽는 부분 END
        ##### LOGIC START




        if isTracker == False:
            tracker = select_ROI(color_image)
            isTracker = True
        color_image,depth = tracking(color_image,depth_frame,tracker)
        cv2.putText(color_image ,"{}cm".format(depth*100), (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
        

        ##### LOGIC END
        # 화면에 보여주는 부분 START
        cv2.namedWindow(MAIN_SCREEN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(MAIN_SCREEN, click,param=depth)
        cv2.imshow(MAIN_SCREEN, color_image)
        # 화면에 보여주는 부분 END

        # q 키 누르면 끄는 코드
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pass