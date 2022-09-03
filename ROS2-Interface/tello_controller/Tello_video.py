from threading import Thread
import sys
import cv2
from queue import Queue
from djitellopy import tello


class FileVideoStreamTello:

    def __init__(self, tello: tello.Tello, queuesize=4):
        """
            Initialize.
            @tello : the drone itself.
            @queuesize : the maximum number of frames the queue can hold.
                        Default - 128
        """
        self.tello = tello
        self.tello.streamon()

        self.stopped = False

        self.q = Queue(maxsize=queuesize)

    def start(self):
        """
            Starts the deamon thread.
        """
        t = Thread(target=self.update, args=())
        # t.deamon = True
        t.start()

        return self

    def update(self):
        """
            While the queue isnt full, read more frames from the drone
            and decode them.
        """

        while True:

            if not self.stopped:

                if not self.q.full():
                    # make sure the queue isnt full
                    frame = self.tello.get_frame_read().frame
                    grabbed = self.tello.get_frame_read().grabbed

                    if not grabbed:
                        print("fail")
                        return
                    
                    self.q.put(frame)

    def read(self):
        """
            return the next frame in the Queue.
        """
        return self.q.get()

    def stop(self):
        """
            Stops the reading from the drone
        """
        self.stopped = True
    
    def more(self):
        """
            Returns True if the queue isnt empty.
            else, False
        """
        return not self.q.empty()

                