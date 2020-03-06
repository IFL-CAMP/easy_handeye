#!/usr/bin/env python2

import rospy
import tf2_ros
import geometry_msgs.msg
from easy_handeye.handeye_calibration import HandeyeCalibration, HandeyeCalibrationParameters


def to_parameters(calibration):
    """
    Populates the given HandeyeCalibration object with the values retrieved from ROS parameters in the namespace of the current node.

    :type calibration: HandeyeCalibration
    :rtype: None
    """
    calib_dict = calibration.to_dict()

    root_params = ['eye_on_hand', 'tracking_base_frame']
    if calib_dict['eye_on_hand']:
        root_params.append('robot_effector_frame')
    else:
        root_params.append('robot_base_frame')

    for rp in root_params:
        rospy.set_param(rp, calib_dict[rp])

    transf_params = 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'

    for tp in transf_params:
        rospy.set_param('transformation/'+tp, calib_dict['transformation'][tp])


def from_parameters(calibration):
    """
    Stores the given calibration as ROS parameters in the namespace of the current node.

    :type calibration: HandeyeCalibration
    :rtype: None
    """
    calib_dict = {}

    root_params = ['eye_on_hand', 'tracking_base_frame']
    for rp in root_params:
        calib_dict[rp] = rospy.get_param(rp)

    if calib_dict['eye_on_hand']:
        calib_dict['robot_effector_frame'] = rospy.get_param('robot_effector_frame')
    else:
        calib_dict['robot_base_frame'] = rospy.get_param('robot_base_frame')

    transf_params = 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
    calib_dict['transformation'] = {}
    for tp in transf_params:
        calib_dict['transformation'][tp] = rospy.get_param('transformation/'+tp)

    calibration.from_dict(calib_dict)


rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

inverse = rospy.get_param('inverse')

calib = HandeyeCalibration(handeye_parameters=HandeyeCalibrationParameters(namespace=rospy.get_namespace()))
calib.from_file()

if calib.parameters.eye_on_hand:
    overriding_robot_effector_frame = rospy.get_param('robot_effector_frame')
    if overriding_robot_effector_frame != "":
        calib.transformation.header.frame_id = overriding_robot_effector_frame
else:
    overriding_robot_base_frame = rospy.get_param('robot_base_frame')
    if overriding_robot_base_frame != "":
        calib.transformation.header.frame_id = overriding_robot_base_frame
overriding_tracking_base_frame = rospy.get_param('tracking_base_frame')
if overriding_tracking_base_frame != "":
    calib.transformation.child_frame_id = overriding_tracking_base_frame

rospy.loginfo('loading calibration parameters into namespace {}'.format(
    rospy.get_namespace()))
to_parameters(calib)

orig = calib.transformation.header.frame_id  # tool or base link
dest = calib.transformation.child_frame_id  # tracking_base_frame

broadcaster = tf2_ros.StaticTransformBroadcaster()
static_transformStamped = geometry_msgs.msg.TransformStamped()

static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = orig
static_transformStamped.child_frame_id = dest

static_transformStamped.transform = calib.transformation.transform

broadcaster.sendTransform(static_transformStamped)
rospy.spin()
