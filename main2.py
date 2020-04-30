from CV_class import CV
import cv2
import math
c = CV()

tracker1,origin_rect_1 = c.select_ROI()
tracker2,origin_rect_2 = c.select_ROI()

while True:
    c._get_frame()
    depth,tracking_box_1 = c.tracking(tracker1)
    depth,tracking_box_2 = c.tracking(tracker2)
    # logic start
    x1 = origin_rect_1[0]
    x1 += origin_rect_1[2] / 2
    x2 = tracking_box_1[0]
    x2 += tracking_box_1[2] / 2
    fx,fy = c.get_fov()
    color_frame = c.get_color_frame()
    dx = (abs(x1-x2)) * 2 * math.tan(math.radians(fx / 2)) * depth / color_frame.width
    dx *= 1000
    # logic end
    c.write_text("{}mm / depth : {}".format(dx,depth))
    c.show_img()
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break
