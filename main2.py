from CV_class import CV
import cv2
import math


c = CV()


c.select_ROI()

while True:
    c._get_frame()
    depth,tracking_box = c.tracking()
    # logic start
    x1 = c.origin_rect[0]
    x1 += c.origin_rect[2] / 2
    x2 = tracking_box[0]
    x2 += tracking_box[2] / 2
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
