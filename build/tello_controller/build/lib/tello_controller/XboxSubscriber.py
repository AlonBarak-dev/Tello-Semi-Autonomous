from numpy import imag
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .followobject import FollowObject

from socket import *
from djitellopy import tello
from threading import Thread
import cv2
import argparse
import imutils
import sys
from math import atan2, cos, sin, sqrt, pi
import numpy as np

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # input arguments
        parser = argparse.ArgumentParser(description='Tello Object tracker. keys: t-takeoff, l-land, v-video, q-quit w-up, s-down, a-ccw rotate, d-cw rotate\n')
        parser.add_argument('-model', type=str, help='DNN model caffe or tensorflow, see data folder', default='')
        parser.add_argument('-proto', type=str, help='Prototxt file, see data folder', default='')
        parser.add_argument('-obj', type=str, help='Type of object to track. [Face, Person], default = Face', default='Face')
        parser.add_argument('-dconf', type=float, help='Detection confidence, default = 0.7', default=0.4)
        parser.add_argument('-debug', type=bool, help='Enable debug, lists messages in console', default=False)
        parser.add_argument('-video', type=str, help='Use as inputs a video file, no tello needed, debug must be True', default="")
        parser.add_argument('-vsize', type=list, help='Video size received from tello', default=(640,480))
        parser.add_argument("-type", type=str,default="DICT_5X5_100", help="type of ArUCo tag to detect")
        parser.add_argument('-th', type=bool, help='Horizontal tracking', default=False)
        parser.add_argument('-tv', type=bool, help='Vertical tracking', default=True)
        parser.add_argument('-td', type=bool, help='Distance tracking', default=True)
        parser.add_argument('-tr', type=bool, help='Rotation tracking', default=True)
        self.args = parser.parse_args()

        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }

        # ROS Joy subscriber
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

        self._rate = self.create_rate(10) # 10 Hz
        
        self.me = tello.Tello()
        self.me.connect()   # connect to the Drone
        self.me.streamon()
        # prints the Battery percentage
        print("Battery percentage:", self.me.get_battery())
        # create the video capture thread
        self.video_thread = Thread(target=self.video)
        # if the battery is too low its arise an error
        if self.me.get_battery() < 10:
            raise RuntimeError("Tello rejected attemp to takeoff due to low Battery")
        
        # self.me.takeoff()
        self.video_thread.start()
    
    def drawAxis(self, img, p_, q_, color, scale):
        p = list(p_)
        q = list(q_)
        
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
        cntr = (int(mean[0,0]), int(mean[0,1]))
        ## [pca]
        
        ## [visualization]
        # Draw the principal components
        cv2.circle(img, cntr, 3, (255, 0, 255), 2)
        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        self.drawAxis(img, cntr, p1, (255, 255, 0), 1)
        self.drawAxis(img, cntr, p2, (0, 0, 255), 5)
        
        angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        ## [visualization]
        
        # Label with the rotation angle
        label = "  Rotation Angle: " + str(-int(np.rad2deg(angle)) - 90) + " degrees"
        textbox = cv2.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
        cv2.putText(img, label, (cntr[0], cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        
        return angle
        

    def listener_callback(self, msg:Joy):
        """
            This is a Callback function for the ROS Joy topic.
            once the controller publish data to the topic, 
            this method sends commands to the Drone base on 
            the data received from the controller.
        """
        big_factor = 100
        medium_factor = 50
        # extract the data from the controller
        data = list(msg.axes)
        a = -data[2] * big_factor    # Yaw
        b = data[3] * big_factor     # Forward / Backward
        c = data[1] * medium_factor  # Up / Down
        d = -data[0] * big_factor    # Left / Right
        
        data = list(msg.buttons)
        land = data[0]      # A
        takeoff = data[3]   # X
        emergency = data[2] # Y
        battery = data[1]   # B
        
        if land != 0:
            print("LAND")
            self.me.land()
        elif takeoff != 0:
            print("TAKEOFF")
            self.me.takeoff()
        elif battery != 0:
            print("Battery percentage:", self.me.get_battery())
        elif emergency != 0:
            try:
                print("EMERGENCY")
                self.me.emergency()
            except Exception as e:
                print("Did not receive OK, reconnecting to Tello")
                self.me.connect()

        else:
            self.me.send_rc_control(int(a), int(b), int(c), int(d))

    def detect_aruco(self, img):
        """
            This method detect ArUco code from all types.
            It detect its Value, Boundaries, Center point and type.
            it return an image with the draws.
        """

        image = imutils.resize(img, width=600)
        # verify that the supplied ArUCo tag exists and is supported by
        # OpenCV
        if self.ARUCO_DICT[self.args.type] is None:
            print("[INFO] ArUCo tag of '{}' is not supported".format(
                self.args.type))
            sys.exit(0)
        # load the ArUCo dictionary, grab the ArUCo parameters, and detect
        # the markers
        for dict in self.ARUCO_DICT:
            # arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.args.type])
            arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[dict])
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
                print("[INFO] detecting '{}' tags...".format(self.ARUCO_DICT[dict]))
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    # draw the bounding box of the ArUCo detection
                    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                    # compute and draw the center (x, y)-coordinates of the ArUco
                    # marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                    # draw the ArUco marker ID on the image
                    cv2.putText(image, str(markerID),
                        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                    print("[INFO] ArUco marker ID: {}".format(markerID))
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
        # Convert image to binary
        _, bw = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        
        # Find all the contours in the thresholded image
        contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        
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
        
        return image


    def video(self):
        """
            This method detects Faces/Persons and Aruco Codes.
            It plot the video captured from the Drone and it's detected objects boundaries.
        """
        fobj = FollowObject(self.me, MODEL=self.args.model, PROTO=self.args.proto, CONFIDENCE=self.args.dconf, DEBUG=False, DETECT=self.args.obj)
        fobj.set_tracking( HORIZONTAL=self.args.th, VERTICAL=self.args.tv,DISTANCE=self.args.td, ROTATION=self.args.tr)

        while True:

            try:
                img = self.me.get_frame_read().frame
                # wait for valid frame
                if img is None: continue
                imghud = img.copy()
                fobj.set_image_to_process(img)
                
            except Exception:
                break
            
            imghud = self.detect_aruco(imghud)
            fobj.draw_detections(imghud, ANONIMUS=False)
            cv2.imshow("TelloCamera",imghud)
            k = cv2.waitKey(1)


def main(args=None):


    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()