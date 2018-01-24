#!/usr/bin/env python2

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from geometry_msgs.msg import Pose2D

from BackgroundSegmentator import BackgroundSegmentator

from ObjectDetector import ObjectDetector, MinimumAreaFilter, BoundingBoxFilter, DetectedObject
from ObjectTracker import ObjectTracker

import cv2

import numpy as np
from matplotlib.cm import flag as cm


class ObjectDetectorNode:

    def __init__(self, imageTopic, segmentedTopic):
        self.imageTopic = imageTopic
        self.segmentedTopic = segmentedTopic
        self.imageSubscriber = rospy.Subscriber(
            imageTopic, Image, callback=self.callback)
        self.cvbridge = CvBridge()
        self.imagePublisher = rospy.Publisher(
            segmentedTopic, Image, queue_size=100)
        self.backgroundSegmentator = BackgroundSegmentator()

        self.objectDetector = ObjectDetector(
            filters=[MinimumAreaFilter(300)])

        self.objectTracker = ObjectTracker()

    def callback(self, msg):
        img = self.cvbridge.imgmsg_to_cv2(msg, 'bgr8')

        fg = self.backgroundSegmentator(img)

        detectedObjects = self.objectDetector(fg)

        for detectedObject in detectedObjects:
            self.objectTracker.track_object(detectedObject)

        self.objectTracker.update()

        for i, detectedObject in enumerate(self.objectTracker.objects):

            if detectedObject.lifetime > 50:
                continue

            cx, cy = detectedObject.centroid

            is_circle = detectedObject.detectedObject.is_circle

            cv2.line(img, (cx - 3, cy - 3), (cx + 3, cy + 3), (0, 0, 255), 2)
            cv2.line(img, (cx + 3, cy - 3), (cx - 3, cy + 3), (0, 0, 255), 2)

            cv2.putText(img, str(i), (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

            if is_circle:
                s = 'c(r={})'.format(int(detectedObject.detectedObject.radius))
            else:
                s = 'r(w={},h={},a={})'.format(
                    int(detectedObject.detectedObject.rectFit[0]), int(detectedObject.detectedObject.rectFit[1]), int(detectedObject.detectedObject.rectFit[2] * 100))
            cv2.putText(img, s, (cx + 10, cy + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

            if is_circle:
                r = detectedObject.detectedObject.radius
                r = int(r)
                cv2.circle(img, (cx, cy), r, (255, 0, 0), 2)
            else:
                rect = detectedObject.detectedObject.rectCntFit
                cv2.drawContours(img, [rect], 0, (255, 0, 0), 2)

        out = self.cvbridge.cv2_to_imgmsg(img, "bgr8")

        self.imagePublisher.publish(out)


if __name__ == '__main__':

    rospy.init_node('object_detector', anonymous=True)

    ObjectDetectorNode('camera/rgb/image_raw', 'segmented_image')

    rospy.spin()
