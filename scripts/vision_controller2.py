#!/usr/bin/env python3

import rospy
import cv2
import pyrealsense2
import face_recognition
import numpy as np
from robotics_final_project.msg import VisionCoords, ListVisionCoords
import os

#os.chdir('~/catkin_ws/src/robotics_final_project/scripts')
# https://pysource.com/2021/03/11/distance-detection-with-depth-camera-intel-realsense-d435i/
from realsense_depth import *

class ObjectDetector(object):
    def __init__(self):

        #initialize ros node
        rospy.init_node('wscr_vision')

        # Initialize Camera Intel Realsense
        self.dc = DepthCamera()

        # Load a sample picture and learn how to recognize it.
        self.chris_rock_image = face_recognition.load_image_file("Chris_Rock.jpg")
        self.chris_rock_encoding = face_recognition.face_encodings(self.chris_rock_image)[0]

        self.jada_img = face_recognition.load_image_file("jada_smith.jpeg")
        self.jada_encoding = face_recognition.face_encodings(self.jada_img)[0]
        
        # Create arrays of known face encodings and their names
        self.known_face_encodings = [
            self.chris_rock_encoding,
            self.jada_encoding
        ]
        self.known_face_names = [
            "Chris Rock",
            "Jada"
        ]

        # List of object positions
        self.positions = []

        #setup publisher to publish x,y, and depth 
        self.vision_pub = rospy.Publisher('/robot_vision', ListVisionCoords, queue_size=10)

        # Allow subscriber time to set up
        rospy.sleep(5)
        
    
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

            # Facial recognition logic adapted from:
            # https://github.com/ageitgey/face_recognition/blob/master/examples/facerec_from_webcam_faster.py

            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(color_frame)
            face_encodings = face_recognition.face_encodings(color_frame, face_locations)

            print(face_locations)
            face_names = []
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                name = "Unknown"

                # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = self.known_face_names[first_match_index]

                face_names.append(name)

            print(face_names)
            jada_found = False
            for (top, right, bottom, left), name in zip(face_locations, face_names):
                # Draw a box around the face
                cv2.rectangle(color_frame, (left, top), (right, bottom), (0, 0, 255), 2)

                # Draw a label with a name below the face
                cv2.rectangle(color_frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                font = cv2.FONT_HERSHEY_DUPLEX
                cv2.putText(color_frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

                if name == "Jada":
                    jada_found = True

                # If face is Chris Rock, find center coordinates and depth
                if name == "Chris Rock":
                    cX = int(left + 0.5 * (right - left))
                    cY = int(top + 0.5 * (bottom - top))
                    cv2.circle(color_frame, (cX, cY), 10, (320, 159, 22), -1)

                    distance = depth_frame[cY,cX]
                    self.positions.append({"x":cX, "y":cY, "dis":distance})
                    cv2.putText(color_frame, "{}mm".format(distance), (cX, cY - 20), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
            if not jada_found:
                print("jada not found")
                self.positions.clear()

            #cv2.imshow('color frame', color_frame)

            # Hit 'q' on the keyboard to quit!
            #if cv2.waitKey(1) & 0xFF == ord('q'):
            #    break
            print(self.positions)
            
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
            #cv2.imshow('color frame', color_frame)
            rospy.sleep(6)



if __name__ == '__main__':
    # declare the ROS node and run it
    ObjectDetector1 = ObjectDetector()
    print("running")
    ObjectDetector1.run()