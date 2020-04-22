from CV_class import CV
import cv2
c = CV()


c.select_ROI()

while True:
    c._get_frame()
    depth,box = c.tracking()
    c.write_text("{}mm / depth : {}".format(depth))
    c.show_img()
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break
