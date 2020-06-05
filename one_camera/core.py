import cv2

OPENCV_OBJECT_TRACKERS = {
    "csrt": cv2.TrackerCSRT_create,
    "kcf": cv2.TrackerKCF_create,
    "boosting": cv2.TrackerBoosting_create,
    "mil": cv2.TrackerMIL_create,
    "tld": cv2.TrackerTLD_create,
    "medianflow": cv2.TrackerMedianFlow_create,
    "mosse": cv2.TrackerMOSSE_create
}

def cv_init(cap):
    frame_count = 1
    ret, img = cap.read()
    rect = cv2.selectROI("select ROI",img)
    tracker = OPENCV_OBJECT_TRACKERS['csrt']()
    tracker.init(img,rect)
    x = 0
    y = 0
    for _ in range(frame_count):
        _, box = tracker.update(img)
        left,top,w,h = [int(v) for v in box]
        right = left + w
        bottom = top + h
        x_middle = (left + right) // 2
        y_middle = (top + bottom) // 2
        x += x_middle
        y += y_middle 
    return tracker,x//frame_count,y//frame_count

