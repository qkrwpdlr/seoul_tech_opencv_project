import cv2
import numpy as np
from core import cv_init,make_ROI_IMG
from matplotlib import pyplot as plt
plt.axis([0, 2000, 0, 2000])



# open video file
video_path = 'test.MTS'
cap = cv2.VideoCapture(video_path)

make_ROI_IMG(cap)

cap.release()
