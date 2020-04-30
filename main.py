import pyrealsense2 as rs
import numpy as np
import cv2
from core import rsInit,select_ROI,tracking,click,select_ROI2
import math
import matplotlib.pyplot as plt
import time
MAIN_SCREEN = "RealSense"
pipeline,pipe_profile  = rsInit()
isTracker = False
origin_point = []
count = 0
datas = []
prevTime = 0

try:
    while True:
        count += 1
        # 카메라 읽는 부분 STA  RT
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
            tracker, rect = select_ROI2(color_image)
            isTracker = True
            origin_point = rect

        color_image,depth,box,depth_image = tracking(color_image,depth_frame,tracker,depth_image,pipe_profile)

        ## origin 으로 부터의 거리
        a = origin_point[0]
        a += origin_point[2] / 2
        b = box[0]
        b += box[2] / 2
        fx,fy = rs.rs2_fov(color_frame.profile.as_video_stream_profile().intrinsics)
        # print(math.tan(math.radians(fx / 2)),origin_point[0] , box[0])
        # print(depth)
        realWdith = (abs(a-b)) * 2 * math.tan(math.radians(fx / 2)) * depth / color_frame.width
        cv2.putText(color_image ,"{}mm Depth : {}".format(realWdith*1000,depth), (0, 10),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))
        datas.append(realWdith*10)
        ## origin 까지의 거리 표시

        curTime = time.time()
        sec = curTime - prevTime
        prevTime = curTime
        fps = 1/ sec

        cv2.putText(color_image ,"fps : {}".format(fps), (0, 100),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0))

        scale = pipe_profile.get_device().first_depth_sensor().get_depth_scale()
        ##### LOGIC END
        # 화면에 보여주는 부분 START
        cv2.namedWindow(MAIN_SCREEN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(MAIN_SCREEN, click,param=[depth_frame,color_frame,color_image])
        cv2.imshow(MAIN_SCREEN, color_image)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.namedWindow("MAIN_SCREEN", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("MAIN_SCREEN", depth_colormap)
        # 화면에 보여주는 부분 END

        # q 키 누르면 끄는 코드
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pass
plt.plot([i for i in range(count)],datas)
plt.show()
