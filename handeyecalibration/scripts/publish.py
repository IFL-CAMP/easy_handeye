#!/usr/bin/env python2

__author__ = 'marco'

import rospy
from tf import TransformBroadcaster, TransformerROS, transformations as tfs
from geometry_msgs.msg import Transform

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

eye_on_hand = rospy.get_param('eye_on_hand')
inverse = rospy.get_param('inverse')

orig = None
dest = None 

# TODO: use HandeyeCalibration class

if eye_on_hand:
    orig = rospy.get_param('tool_frame')
    dest = rospy.get_param('optical_origin_frame')
    prefix = 'tool_to_camera'
else:
    orig = rospy.get_param('base_link_frame')
    dest = rospy.get_param('optical_origin_frame')
    prefix = 'base_to_camera'

translation = (rospy.get_param(prefix + '_x'),
               rospy.get_param(prefix + '_y'),
               rospy.get_param(prefix + '_z'))
rotation = (rospy.get_param(prefix + '_qx'),
            rospy.get_param(prefix + '_qy'),
            rospy.get_param(prefix + '_qz'),
            rospy.get_param(prefix + '_qw'))

if inverse:
    transformer = TransformerROS()
    result_tf = Transform(translation, rotation)
    cal_mat = transformer.fromTranslationRotation(result_tf.translation, result_tf.rotation)
    cal_mat_inv = tfs.inverse_matrix(cal_mat)
    translation = tfs.translation_from_matrix(cal_mat_inv)
    rotation = tfs.quaternion_from_matrix(cal_mat_inv)

    orig, dest = dest, orig

rospy.loginfo('publishing transformation ' + orig + ' -> ' + dest + ':\n' + str((translation, rotation)))

broad = TransformBroadcaster()

rate = rospy.Rate(50)

while not rospy.is_shutdown():
    broad.sendTransform(translation, rotation, rospy.Time.now(), dest, orig)
    rate.sleep()