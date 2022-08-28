import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .followobject import FollowObject

from socket import *
from djitellopy import tello
from threading import Thread
import cv2
import argparse

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
        parser.add_argument('-th', type=bool, help='Horizontal tracking', default=False)
        parser.add_argument('-tv', type=bool, help='Vertical tracking', default=True)
        parser.add_argument('-td', type=bool, help='Distance tracking', default=True)
        parser.add_argument('-tr', type=bool, help='Rotation tracking', default=True)
        self.args = parser.parse_args()


        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)

        self._rate = self.create_rate(10) # 10 Hz
        
        self.me = tello.Tello()
        self.me.connect()
        self.me.streamon()
        
        print("Battery percentage:", self.me.get_battery())
        self.video_thread = Thread(target=self.video)

        if self.me.get_battery() < 10:
            raise RuntimeError("Tello rejected attemp to takeoff due to low Battery")
        
        # self.me.takeoff()
        self.video_thread.start()

    def listener_callback(self, msg:Joy):
        big_factor = 100
        medium_factor = 50

        data = list(msg.axes)
        a = -data[2] * big_factor    # Yaw
        b = data[3] * big_factor     # Forward / Backward
        c = data[1] * medium_factor  # Up / Down
        d = -data[0] * big_factor    # Left / Right
        
        data = list(msg.buttons)
        land = data[0]      # A
        takeoff = data[3]   # Y
        emergency = data[2] # X
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

    def video(self):
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