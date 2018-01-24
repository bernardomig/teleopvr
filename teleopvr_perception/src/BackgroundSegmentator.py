#!/usr/bin/env python2

import numpy as np
import cv2


class BackgroundSegmentator:

    def __init__(self, dev=1.0, kernel_size=3, opening_iterations=2, closing_iterations=2):
        self.dev = dev
        self.kernel_size = kernel_size
        self.opening_iterations = opening_iterations
        self.closing_iterations = closing_iterations

    def __call__(self, img):

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]

        mean = np.mean(h)
        dev = self.dev * np.std(h)

        bg_range = [mean - dev, mean + dev]
        bg = np.logical_and(h > bg_range[0], h < bg_range[1])

        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        erosion = cv2.morphologyEx(
            np.uint8(bg) * 255, cv2.MORPH_OPEN, kernel, iterations=self.opening_iterations)

        closing = cv2.morphologyEx(
            erosion, cv2.MORPH_CLOSE, kernel, iterations=self.closing_iterations)

        _, fg = cv2.threshold(closing, 127, 255, cv2.THRESH_BINARY_INV)

        bg = 255 - fg

        _, contours, _ = cv2.findContours(
            bg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        biggest_cnt = sorted([(cv2.contourArea(cnt), cnt)
                              for cnt in contours], key=lambda x: x[0])[-1][1]

        fg_mask = np.zeros(fg.shape)

        cv2.drawContours(fg_mask, [biggest_cnt], 0, 255, cv2.FILLED)

        fg[~(fg_mask > 0)] = 0

        return fg
