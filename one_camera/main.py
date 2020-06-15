import cv2
import numpy as np
from core import cv_init
from matplotlib import pyplot as plt
plt.axis([0, 2000, 0, 2000])



# open video file
video_path = 'test.MTS'
cap = cap = cv2.VideoCapture(video_path)

tracker,origin_x,origin_y = cv_init(cap)

init_frame = 5
while True:
    ret, img = cap.read()
    _, box = tracker.update(img)
    left,top,w,h = [int(v) for v in box]
    right = left + w
    bottom = top + h
    x_middle = (left + right) // 2
    y_middle = (top + bottom) // 2    
    # cv2.rectangle(img, (left,top),(right,bottom), (255,255,255) ,3)
    gap_x = origin_x - x_middle
    gap_y = origin_y - y_middle
    ## logic
    ## logic
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edge = cv2.Canny(img,100,200)
    circles = cv2.HoughCircles(gray[top:bottom,left:right],cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=60,minRadius=0,maxRadius=0)
    if circles is not None: 
        circles = np.uint16(np.around(circles))
        for c in circles[0,:]:
            center = (c[0] + left,c[1] + top)
            radius = c[2]
            
            cv2.circle(img,center,radius,[0,255,0],2)
    cv2.imshow('img', img)
    if cv2.waitKey(1) == ord('q'): break

cap.release()
