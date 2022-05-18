import cv2
import pyrealsense2

# https://pysource.com/2021/03/11/distance-detection-with-depth-camera-intel-realsense-d435i/
from realsense_depth import *

class ObjectDetector:
    def __init__(self):
        # Initialize Camera Intel Realsense
        dc = DepthCamera()

        # List of object positions
        positions = []

    def run(self):
        while True:
            ret, depth_frame, color_frame = dc.get_frame()

            # https://stackoverflow.com/questions/60486029/how-to-find-the-center-of-black-objects-in-an-image-with-python-opencv
            # grayscale, Gaussian blur, Otsu's threshold
            gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (3,3), 0)
            thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

            # Find contours
            ROI_number = 0
            cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]

            for c in cnts:
                # Obtain bounding rectangle to get measurements
                # x,y,w,h = cv2.boundingRect(c)

                # Find centroid
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # # Crop and save ROI
                # ROI = original[y:y+h, x:x+w]
                # cv2.imwrite('ROI_{}.png'.format(ROI_number), ROI)
                # ROI_number += 1

                # Draw the contour and center of the shape on the image
                # cv2.rectangle(image,(x,y),(x+w,y+h),(36,255,12), 4)
                cv2.circle(image, (cX, cY), 10, (320, 159, 22), -1)
                distance = depth_frame[cY,cX]
                coordinates.append({"x":cX, "y":cY, "dis":distance})

                cv2.putText(color_frame, "{}mm".format(distance), (cX, cY - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

                key = cv2.waitKey(1)
                if key == 27:
                    break

if __name__ == '__main__':
    # declare the ROS node and run it
    ObjectDetector1 = ObjectDetector()
    print("running")
    ObjectDetector1.run()