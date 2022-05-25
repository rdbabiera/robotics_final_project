#!/usr/bin/env python3

import rospy
import cv2
import pyrealsense2
from robotics_final_project.msg import VisionCoords, ListVisionCoords

# https://pysource.com/2021/03/11/distance-detection-with-depth-camera-intel-realsense-d435i/
from realsense_depth import *

class ObjectDetector(object):
    def __init__(self):

        #initialize ros node
        rospy.init_node('wscr_vision')

        # Initialize Camera Intel Realsense
        self.dc = DepthCamera()

        # List of object positions
        self.positions = []

        #setup publisher to publish x,y, and depth 
        self.vision_pub = rospy.Publisher('/robot_vision', ListVisionCoords, queue_size=10)

        # Allow subscriber time to set up
        rospy.sleep(1)
        
    
    #funtion to publish position from depth camera
    def publish_vision(self, positions):
        pub = ListVisionCoords()
        pub.positions = []
        for pos in positions:
            vision_coords = VisionCoords()
            vision_coords.x = pos["x"]
            vision_coords.y = pos["y"]
            vision_coords.depth = pos["dis"]
            pub.positions.append(vision_coords)
        self.vision_pub.publish(pub)


    def run(self):
        while True:
            #reset positions list
            self.positions.clear()

            ret, depth_frame, color_frame = self.dc.get_frame()

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
                if M['m00'] == 0:
                    continue
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # # Crop and save ROI
                # ROI = original[y:y+h, x:x+w]
                # cv2.imwrite('ROI_{}.png'.format(ROI_number), ROI)
                # ROI_number += 1

                # Draw the contour and center of the shape on the image
                # cv2.rectangle(image,(x,y),(x+w,y+h),(36,255,12), 4)
                cv2.circle(color_frame, (cX, cY), 10, (320, 159, 22), -1)
                distance = depth_frame[cY,cX]
                self.positions.append({"x":cX, "y":cY, "dis":distance})

                cv2.putText(color_frame, "{}mm".format(distance), (cX, cY - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)

                key = cv2.waitKey(1)
               # if key == 27:
                #    break
            
            self.publish_vision(self.positions)
            '''
            max_dis = 0
            max_pos = {}
            for pos in self.positions:
                if pos["dis"] > max_dis:
                    max_dis = pos["dis"]
                    max_pos = pos
            print("max_pos:", max_pos)
            if len(max_pos) != 0:
                print("publishing camera feed")
                self.publish_vision(max_pos)
            '''
            #cv2.imshow('depth frame', depth_frame)
            cv2.imshow('color frame', color_frame)
            rospy.sleep(6)



if __name__ == '__main__':
    # declare the ROS node and run it
    ObjectDetector1 = ObjectDetector()
    print("running")
    ObjectDetector1.run()