#!/usr/bin/python2.7

import rospy
from std_msgs.msg import Empty
from tf import TransformListener

rospy.init_node('signaller')
while rospy.get_time() == 0.0: pass

pub = rospy.Publisher('take_sample',Empty)
listener = TransformListener()

samples = []

while not rospy.is_shutdown():
	c = raw_input('Press ENTER to take sample, s+ENTER to quit and save')
	if c == 's':
		break
	if c == '':
		t = rospy.Time.now()
		pub.publish(Empty())
		# TODO: try combinations of inverses
		listener.waitForTransform('base_link', 'tool0', t, rospy.Duration(3))
		listener.waitForTransform('optical_origin', 'triangle_scalene', t, rospy.Duration(3))
		robot_transform = listener.lookupTransform('base_link', 'tool0', t)
		optical_transform = listener.lookupTransform('optical_origin', 'triangle_scalene', t)
		# robot_transform = listener.lookupTransform('tool0', 'base_link', t)
		# optical_transform = listener.lookupTransform('triangle_scalene', 'optical_origin', t)
		samples.append({'robot': robot_transform, 'optical': optical_transform})
		print('got sample at ROS time; '+t)

print('transforms: ')
for s in samples:
	print s


def blah():
	import pickle
	hec.samples = pickle.load(open('../transforms1.p','rb'))
	hec._inner_to_visp_samples()