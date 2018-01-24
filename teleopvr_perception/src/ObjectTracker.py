#!/usr/bin/env python2

from ObjectDetector import DetectedObject


class TrackedObject:
    def __init__(self, obj):
        self.centroid = obj.centroid
        self.lifetime = 0
        self.detectedObject = obj

    @property
    def lifetime(self):
        return self.lifetime

    def step(self):
        self.lifetime += 1

    def update(self, obj):
        self.detectedObject = obj
        self.lifetime = 0


class ObjectTracker:

    def __init__(self):

        self.objects = []

    def track_object(self, obj):
        mo = self.match_object(obj)
        if mo is None:
            print("detected new object")
            self.objects.append(TrackedObject(obj))
        else:
            self.objects[mo].update(obj)

    def match_object(self, obj):
        for i, tracked in enumerate(self.objects):
            if self.object_likelyhood(tracked.detectedObject, obj):
                return i
        return None

    def object_likelyhood(self, obj1, obj2):
        (c1x, c1y) = obj1.centroid
        (c2x, c2y) = obj2.centroid

        if obj1.is_circle != obj2.is_circle:
            return False

        if abs(c1x - c2x) > 15 or abs(c1y - c2y) > 15:
            return False

        if obj1.is_circle:
            r1 = obj1.radius
            r2 = obj2.radius

            return abs(r1 - r2) / r1 < 0.2
        else:
            w1, h1, a1 = obj1.rectFit
            w2, h2, a2 = obj2.rectFit

            return abs(w1 - w2) / w1 < 0.3 and abs(h1 - h2) / h1 < 0.3 and abs(a1 - a2) / a1 < 0.3

    def update(self):
        for obj in self.objects:
            obj.step()
