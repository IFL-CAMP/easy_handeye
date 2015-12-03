#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformerROS, transformations as tfs
from geometry_msgs.msg import Transform
from handeyecalibration.handeye_calibration import HandeyeCalibration

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

inverse = rospy.get_param('inverse')

calib = HandeyeCalibration()
calib.from_file()
rospy.loginfo('loading calibration parameters into namespace {}'.format(rospy.get_namespace()))
calib.to_parameters()

orig = calib.transformation.header.frame_id
dest = calib.transformation.child_frame_id

transformer = TransformerROS()
result_tf = calib.transformation.transform
transl = result_tf.translation.x, result_tf.translation.y, result_tf.translation.z
rot = result_tf.rotation.x, result_tf.rotation.y, result_tf.rotation.z, result_tf.rotation.z
cal_mat = transformer.fromTranslationRotation(transl, rot)
if inverse:
    cal_mat = tfs.inverse_matrix(cal_mat)
    orig, dest = dest, orig
translation = tfs.translation_from_matrix(cal_mat)
rotation = tfs.quaternion_from_matrix(cal_mat)

rospy.loginfo('publishing transformation ' + orig + ' -> ' + dest + ':\n' + str((translation, rotation)))

broad = TransformBroadcaster()

rate = rospy.Rate(50)

while not rospy.is_shutdown():
    broad.sendTransform(translation, rotation, rospy.Time.now(), dest, orig)
    rate.sleep()
