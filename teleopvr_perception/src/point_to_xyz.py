#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
import tf2_geometry_msgs

import tf2_ros

import numpy as np

rospy.init_node('points_to_xyz_node')

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

cvb = CvBridge()

camera_info_msg = rospy.wait_for_message(
    'camera/depth_registered/camera_info', CameraInfo)

camera_info = PinholeCameraModel()
camera_info.fromCameraInfo(camera_info_msg)

vis_pub = rospy.Publisher('point_markers', Marker, queue_size=10)

i = 0


def pointCallback(msg):
    u, v = int(msg.x), int(msg.y)
    uv = (u, v)
    depth_img_msg = rospy.wait_for_message(
        'camera/depth_registered/image', Image)
    depth_img = cvb.imgmsg_to_cv2(depth_img_msg)
    xyz = camera_info.projectPixelTo3dRay(uv)
    depth = depth_img[v, u]

    xyz = np.array(xyz)
    xyz = xyz / xyz[2] * depth

    print xyz

    marker = Marker()
    marker.header.frame_id = 'camera_rgb_optical_frame'
    marker.header.stamp = rospy.Time()
    marker.action = Marker.ADD
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.pose.position.x = xyz[0]
    marker.pose.position.y = xyz[1]
    marker.pose.position.z = xyz[2]

    vis_pub.publish(marker)


pointSubscriber = rospy.Subscriber(
    '/camera/depth_registered/image_mouse_left', Point, callback=pointCallback, queue_size=10)


rospy.spin()
