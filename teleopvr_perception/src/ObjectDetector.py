#!/usr/bin/env python2

import numpy as np
import cv2


class DetectedObject:

    def __init__(self, contour):
        self.contour = cv2.convexHull(contour)
        self.moments = cv2.moments(self.contour)

    @property
    def area(self):
        return self.moments['m00']

    @property
    def centroid(self):
        cx = int(self.moments['m10'] / self.moments['m00'])
        cy = int(self.moments['m01'] / self.moments['m00'])
        return (cx, cy)

    @property
    def bbox(self):
        x, y, w, h = cv2.boundingRect(self.contour)
        return (x, y, x + w, y + h)

    @property
    def rectCntFit(self):
        rect = cv2.minAreaRect(self.contour)
        rect = cv2.boxPoints(rect)
        return np.int0(rect)

    @property
    def rectFit(self):
        rect = self.rectCntFit
        r1 = rect[0]
        r2 = rect[1]
        r3 = rect[3]
        w = np.linalg.norm(np.array(r1) - np.array(r2))
        h = np.linalg.norm(np.array(r2) - np.array(r3))
        if w > h:
            w, h = h, w
        theta = np.arctan2((r2 - r1)[0], (r2 - r1)[1])
        return (w, h, theta)

    @property
    def radius(self):
        (cx, cy), radius = cv2.minEnclosingCircle(self.contour)
        return radius

    @property
    def perimeter(self):
        return cv2.arcLength(self.contour, True)

    @property
    def shape_factor(self):
        return 4 * np.pi * self.area / (self.perimeter ** 2)

    @property
    def is_circle(self):
        return self.shape_factor > 0.8


class MinimumAreaFilter:
    def __init__(self, area):
        self.area = area

    def __call__(self, detectedObject):
        return detectedObject.area > self.area


class BoundingBoxFilter:
    def __init__(self, bbox):
        self.bbox = bbox

    def __call__(self, detectedObject):
        ox0, oy0, ox1, oy1 = detectedObject.bbox
        x0, y0, x1, y1 = self.bbox
        return ox0 > x0 and ox1 < x1 and oy0 > y0 and oy1 < y1


class ObjectDetector:

    def __init__(self, filters=[]):
        self.filters = filters

    def __call__(self, fg):
        _, contours, _ = cv2.findContours(
            fg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detectedObjects = [DetectedObject(cnt) for cnt in contours]

        for ft in self.filters:
            detectedObjects = filter(ft, detectedObjects)

        return list(detectedObjects)
