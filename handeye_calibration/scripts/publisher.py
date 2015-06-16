#!/usr/bin/env python2

__author__ = 'marco'

import rospy
from tf import TransformBroadcaster

rospy.init_node('handeye_calibration_publisher')
while rospy.get_time() == 0.0: pass

eye_on_hand = rospy.get_param('eye_on_hand')

orig = None
dest = None 

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

rospy.loginfo('publishing transformation ' + orig + ' -> ' + dest + ':\n' + str((translation,rotation)))

broad = TransformBroadcaster()

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    broad.sendTransform(translation, rotation, rospy.Time.now(), dest, orig)  #  FIXME: looks like inverted
    rate.sleep()