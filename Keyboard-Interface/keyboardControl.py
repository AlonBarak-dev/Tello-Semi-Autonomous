from imp import is_frozen_package
from time import sleep
from turtle import down
from numpy import imag
from .Aruco_detection import ArucoDetection
from .followobject import FollowObject
from .Tello_video import FileVideoStreamTello
from socket import *
from djitellopy import tello
from threading import Thread
import cv2
import argparse
from math import atan2, cos, sin, sqrt, pi
import numpy as np
import keyboard

class MinimalSubscriber():

    def __init__(self):

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

        # start the keyboard thread
        self.keyboard_thread = Thread(target=self.keyboard_control)
        self.keyboard_thread.start()

        # connect to the Drone
        self.me = tello.Tello()
        self.me.connect()   

        # prints the Battery percentage
        print("Battery percentage:", self.me.get_battery())
        
        # create the video capture thread
        self.video_thread = Thread(target=self.video)
        # if the battery is too low its arise an error
        if self.me.get_battery() < 10:
            raise RuntimeError("Tello rejected attemp to takeoff due to low Battery")

        # Aruco detector
        self.aruco = ArucoDetection(self.ARUCO_DICT)
        # stream thread
        self.streamQ = FileVideoStreamTello(self.me)
        self.streamQ.start()
        self.video_thread.start()
        

    def keyboard_control(self):
        """
            This method allows the user to control 
            its drone using the keyboard.
            'space' - takeoff / land
            'b' - battery
            'e' - emergancy
            'up' - Up
            'down' - Down
            'left' - left
            'right' - right
            'w' - Forawrd
            's' - Backward
            'a/d' - YAW (ANGLE/DIRECTION)
        """
        big_factor = 100
        medium_factor = 50
        tookoff = False

        while True:
            a, b, c, d = 0, 0, 0, 0

            # Takeoff 
            if keyboard.is_pressed('space') and not tookoff:
                tookoff = True
                self.me.takeoff()
            # Land
            elif keyboard.is_pressed('space') and tookoff:
                tookoff = False
                self.me.land()
            # Battery
            elif keyboard.is_pressed('b'):
                print("Battery percentage:", self.me.get_battery())
            
            # Emergency
            elif keyboard.is_pressed('e'):
                try:
                    print("EMERGENCY")
                    self.me.emergency()
                except Exception as e:
                    print("Did not receive OK, reconnecting to Tello")
                    self.me.connect()
            
            # Up / Down
            elif keyboard.is_pressed('up'):
                a = 0.5 * medium_factor
            elif keyboard.is_pressed("down"):
                a = -0.5 * medium_factor
            
            # Left / Right
            elif keyboard.is_pressed('left'):
                d = -0.5 * big_factor
            elif keyboard.is_pressed('right'):
                d = 0.5 * big_factor
            
            # Forward / Backward
            elif keyboard.is_pressed('w'):
                b = 0.5 * big_factor
            elif keyboard.is_pressed('s'):
                b = -0.5 * big_factor
            
            # YAW
            elif keyboard.is_pressed('a'):
                c = 0.5 * big_factor
            elif keyboard.is_pressed('d'):
                c = -0.5 * big_factor
            
            # send the commands to the drone
            self.me.send_rc_control(int(a), int(b), int(c), int(d))


    def video(self):
        """
            This method detects Faces/Persons and Aruco Codes.
            It plot the video captured from the Drone and it's detected objects boundaries.
        """
        fobj = FollowObject(self.me, MODEL=self.args.model, PROTO=self.args.proto, CONFIDENCE=self.args.dconf, DEBUG=False, DETECT=self.args.obj)
        fobj.set_tracking( HORIZONTAL=self.args.th, VERTICAL=self.args.tv,DISTANCE=self.args.td, ROTATION=self.args.tr)

        while True:
            try:
                img = self.streamQ.read()
                # wait for valid frame
                # if img is None: continue
                imghud = img.copy()
                fobj.set_image_to_process(img)
                self.aruco.set_image_to_process(img)
                ids, corners = self.aruco.draw_detection(image=imghud)
                fobj.draw_detections(imghud,ids, corners, ANONIMUS=False)
                
            except Exception:
                break
            
            cv2.imshow("CleanTello", img)
            cv2.imshow("TelloCamera",imghud)
            k = cv2.waitKey(1)


if __name__ == '__main__':
    tello = MinimalSubscriber()