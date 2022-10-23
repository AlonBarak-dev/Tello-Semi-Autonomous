import threading
from typing import List
import cv2
import argparse
import sys
from math import atan2, cos, sin, sqrt, pi
import numpy as np
import safethread
import imutils

class ArucoDetection:

    def __init__(self, aruco_dict):
        
        self.img = None
        self.ARUCO_DICT = aruco_dict
        self.corners = []
        self.ids = []
        self.contours = []

        self.ticker = threading.Event()
        # processing frequency (to spare CPU time)
        self.cycle_counter = 1
        self.cycle_activation = 10


        self.detection = safethread.SafeThread(target=self.detect_aruco).start()

    def set_image_to_process(self, img):
        """Image to process

        Args:
            img (nxmx3): RGB image
        """
        self.img = img
    
    def detect_aruco(self):
        """
            This method detect ArUco code from all types.
            It detect its Value, Boundaries, Center point and type.
            it return an image with the draws.
        """
        # time base
        self.ticker.wait(0.005)

        if self.img is not None:
            img = self.img.copy()

            image = imutils.resize(img, width=720)
            # load the ArUCo dictionary, grab the ArUCo parameters, and detect
            # the markers
            
            corners_list = []
            ids_list = []
            # for dict in self.ARUCO_DICT:
            # arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.args.type])
            arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT["DICT_4X4_100"])
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
            if ids is not None:
                for c, i in zip(corners, ids):
                    ids_list.append(i)
                    corners_list.append(c)
                    print("Code: ", i , " Coords: ", c)
                    
            self.corners = corners_list
            self.ids = ids_list
        
        self.cycle_counter +=1


    def draw_detection(self, image):
        """
            This method draw the latest detections on the given image.
        """
        # contours = self.contours
        # for i, c in enumerate(contours):
        
        #     # Calculate the area of each contour
        #     area = cv2.contourArea(c)
            
        #     # Ignore contours that are too small or too large
        #     if area < 3700 or 100000 < area:
        #         continue
            
        #     # Draw each contour only for visualisation purposes
        #     cv2.drawContours(image, contours, i, (0, 0, 255), 2)
            
        #     # Find the orientation of each shape
        #     self.getOrientation(c, image)

        return self.ids, self.corners


