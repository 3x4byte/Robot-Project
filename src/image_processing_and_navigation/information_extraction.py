import cv2
import numpy as np


class InformationExtraction:

    def get_graymap(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def get_diff_as_bitmap(self, img_clear, img):
        img = cv2.subtract(img_clear, img)
        img = self.get_graymap(img)
        _, img = cv2.threshold(img, 15, 255, cv2.THRESH_BINARY)
        return img

    def erode_dilate(self, img):
        kernel = np.ones((5, 5), np.uint8)
        img = cv2.erode(img, kernel, iterations=1)
        img = cv2.dilate(img, kernel, iterations=2)
        return img

    def get_contours(self, img):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours, hierarchy

    def get_robot(self, contours):
        for i in range(len(contours)):
            if 4000 < cv2.contourArea(contours[i]) < 9000:
                bot_x, bot_y, bot_w, bot_h = cv2.boundingRect(contours[i])
                return [bot_x, bot_y, bot_w, bot_h]

    def get_robot_center(self, contours):
        for i in range(len(contours)):
            if 4000 < cv2.contourArea(contours[i]) < 9000:
                bot_x, bot_y, bot_w, bot_h = cv2.boundingRect(contours[i])
                return [bot_x+bot_w/2, bot_y+bot_h/2]

    def get_bricks(self, contours):
        bricks = []
        for i in range(len(contours)):
            if 10 < cv2.contourArea(contours[i]) < 250:
                brick_x, brick_y, brick_w, brick_h = cv2.boundingRect(contours[i])
                bricks.append([brick_x, brick_y, brick_w, brick_h])
        return bricks

    def get_bricks_center(self, contours):
        bricks = []
        for i in range(len(contours)):
            if 10 < cv2.contourArea(contours[i]) < 250:
                brick_x, brick_y, brick_w, brick_h = cv2.boundingRect(contours[i])
                bricks.append([brick_x+brick_w/2, brick_y+brick_h/2])
        return bricks

