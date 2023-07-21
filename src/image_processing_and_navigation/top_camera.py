from threading import Thread
import cv2
import time
import numpy as np


class TopCamera:

    def __init__(self, url):
        self.url = url
        self.is_stopped = False
        self.img = None

    def start(self):
        Thread(target=self._start).start()

    def _start(self):
        self.cap = cv2.VideoCapture(self.url)
        while not self.is_stopped:
            _, self.img = self.cap.read()
            cv2.waitKey(10)

    def start_show(self):
        Thread(target=self._start_show).start()

    def _start_show(self):
        self.cap = cv2.VideoCapture(self.url)
        while not self.is_stopped:
            _, self.img = self.cap.read()
            cv2.imshow("camera", self.img)
            cv2.waitKey(10)

    def stop(self):
        self.is_stopped = True
        time.sleep(0.5)
        self.cap.release()

    def get_img(self):
        return self.img

    def get_img_undistorted(self, DIM, K, D):
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        img = cv2.remap(self.img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return img
