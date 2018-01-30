#!/usr/bin/env python2

ARUCO_ID = 10
CAMERA_FRAME = 'camera_base_link'
ROBOT_BASE_FRAME = 'robot_eef_tool_tip'


import rospy
import tf2_ros
import tf
import geometry_msgs.msg
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np


from visp_hand2eye_calibration.msg import TransformArray

rospy.init_node('hand2eye_calibration_node')

tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)

tf_fid_broadcaster = tf2_ros.TransformBroadcaster()

# rospy.sleep(1)

# world_effector_pub = rospy.Publisher(
#     'world_effector', geometry_msgs.msg.Transform, queue_size=1)
# camera_object_pub = rospy.Publisher(
#     'camera_object', geometry_msgs.msg.Transform, queue_size=1)

# print tf_listener.lookupTransform('eef_tool_tip', 'robot_base_link', rospy.Time())

# f = rospy.wait_for_message('fiducial_transforms', FiducialTransformArray, 100)


def update_fid(msg):

    fid_transform = msg.transforms[0].transform

    fid = geometry_msgs.msg.TransformStamped()
    fid.header.frame_id = 'camera_rgb_optical_frame'
    fid.child_frame_id = 'calibration_object'
    fid.transform.translation.x = fid_transform.translation.x
    fid.transform.translation.y = fid_transform.translation.y
    fid.transform.translation.z = fid_transform.translation.z
    fid.transform.rotation.x = fid_transform.rotation.x
    fid.transform.rotation.y = fid_transform.rotation.y
    fid.transform.rotation.z = fid_transform.rotation.z
    fid.transform.rotation.w = fid_transform.rotation.w

    tf_fid_broadcaster.sendTransform(fid)


rospy.Subscriber('fiducial_transforms', FiducialTransformArray,
                 update_fid, queue_size=10)

print 'system ready...'
print 'press <enter> to register the poses'
print 'type <y> to finish the capturing'

rotations = []
translations = []

while raw_input('') != 'y':

    robot = tfBuffer.lookup_transform(
        'robot_base_link', 'eef_tool_tip', rospy.Time())
    fiducial = tfBuffer.lookup_transform(
        'calibration_object', 'camera_link', rospy.Time())

    # world_effector_pub.publish(robot.transform)
    # camera_object_pub.publish(fiducial.transform)

    print robot.transform
    print fiducial.transform

    T1 = robot.transform.translation
    T2 = fiducial.transform.translation
    T1 = tf.transformations.translation_matrix([T1.x, T1.y, T1.z])
    T2 = tf.transformations.translation_matrix([T2.x, T2.y, T2.z])
    R1 = robot.transform.rotation
    R2 = fiducial.transform.rotation
    R1 = tf.transformations.quaternion_matrix([R1.x, R1.y, R1.z, R1.w])
    R2 = tf.transformations.quaternion_matrix([R2.x, R2.y, R2.z, R2.w])
    T = T1.dot(R1).dot(T2).dot(R2)

    print T

    Rot = tf.transformations.euler_from_matrix(T)
    Tr = tf.transformations.translation_from_matrix(T)
    rotations.append(Rot)
    translations.append(Tr)

    print 'xyz = ', Tr
    print 'rpy = ', Rot

print '======================'
print 'Printing Final Results'
print '======================'

rotations = np.array(rotations).mean(axis=0)
translations = np.array(translations).mean(axis=0)

print 'xyz = ', translations
print 'rpy = ', rotations
