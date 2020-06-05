import numpy as np
import cv2
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
fig = plt.figure()
nx = 1
ny = 1
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,i, 700)
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")
print(aruco_dict)
plt.savefig("markers")
plt.show()
