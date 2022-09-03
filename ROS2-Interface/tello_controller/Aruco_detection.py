import threading
from typing import List
import cv2
import argparse
import imutils
import sys
from math import atan2, cos, sin, sqrt, pi
import numpy as np
from . import safethread

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
        self.orientation = safethread.SafeThread(target=self.getOrientation_detection).start()

    def set_image_to_process(self, img):
        """Image to process

        Args:
            img (nxmx3): RGB image
        """
        self.img = img
    
    def drawAxis(self, img, p_, q_, color, scale):
        p = list(p_)
        q = list(q_)
        # print(p, q)
        
        ## [visualization1]
        angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
        hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
        
        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * cos(angle)
        q[1] = p[1] - scale * hypotenuse * sin(angle)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        
        # create the arrow hooks
        p[0] = q[0] + 9 * cos(angle + pi / 4)
        p[1] = q[1] + 9 * sin(angle + pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        
        p[0] = q[0] + 9 * cos(angle - pi / 4)
        p[1] = q[1] + 9 * sin(angle - pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)

    def getOrientation(self, pts, img):
        ## [pca]
        # Construct a buffer used by the pca analysis
        sz = len(pts)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i,0] = pts[i,0,0]
            data_pts[i,1] = pts[i,0,1]
        
        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        
        # Store the center of the object
        # cntr = (int(mean[0,0]), int(mean[0,1]))
        ## [pca]
        
        ## [visualization]
        # Draw the principal components
        # cv2.circle(img, cntr, 3, (255, 0, 255), 2)
        # p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        # p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        # self.drawAxis(img, cntr, p1, (255, 255, 0), 1)
        # self.drawAxis(img, cntr, p2, (0, 0, 255), 5)
        
        angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        ## [visualization]
        
        # Label with the rotation angle
        # label = "  Rotation Angle: " + str(-int(np.rad2deg(angle)) - 90) + " degrees"
        # textbox = cv2.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
        # cv2.putText(img, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        print(angle)
        return angle

    def getOrientation_detection(self):
        """
            This method finds the contours in the image
        """
        # time base
        self.ticker.wait(0.005)

        if self.img is not None:
            image = self.img.copy()
        
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
            # Convert image to binary
            _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            
            # Find all the contours in the thresholded image
            contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            pts_list = []
            for pts in list(contours):
                for box in self.corners:
                    box = box.reshape((4,2))
                    if pts[0] > box[0][0] and pts[0] < box[1][0]:
                        # if pts[1] < box[0][1] and pts[1] > box[3][1]:
                        pts_list.append(pts)
            self.contours = pts_list
        
        self.cycle_counter += 1


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
            for dict in self.ARUCO_DICT:
                # arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.args.type])
                arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[dict])
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
        contours = self.contours
        print(len(contours))
        for i, c in enumerate(contours):
        
            # Calculate the area of each contour
            area = cv2.contourArea(c)
            
            # Ignore contours that are too small or too large
            if area < 3700 or 100000 < area:
                continue
            
            # Draw each contour only for visualisation purposes
            cv2.drawContours(image, contours, i, (0, 0, 255), 2)
            
            # Find the orientation of each shape
            self.getOrientation(c, image)

        return self.ids, self.corners


