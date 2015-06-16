__author__ = 'marco'

import pickle
import rospy
import tf
from geometry_msgs.msg import Transform
import tf.transformations as tfs
import numpy as np
import PyKDL

samples = pickle.load(open('/home/marco/projects/handeye/transforms2.p'))

rospy.init_node('scratch')
while rospy.get_time() == 0.0: pass

b = tf.TransformBroadcaster()


def tf_to_matrix(tm):
	return np.dot(tfs.translation_matrix(tm.translation), tfs.quaternion_matrix(tm.rotation))


def matrix_to_tf(m):
	return Transform(tfs.translation_from_matrix(m), tfs.quaternion_from_matrix(m))

for s in samples:
	t = rospy.Time.now()
	so = s['optical']
	tro = Transform(*so)
	sr = s['robot']
	b.sendTransform(so[0], so[1], 'optical_origin', 'triangle_scalene')

compositions = []
for s in samples:
	tro = Transform(*s['optical'])
	trom = tf_to_matrix(tro)
	tromI = tfs.inverse_matrix(trom)
	trr = Transform(*s['robot'])
	trrm = tf_to_matrix(trr)
	trrmI = tfs.inverse_matrix(trrm)
	# trrI = matrix_to_tf(trrmI)
	tc = np.dot(trom, trrmI)
	compositions.append(tc)

diffs = [compositions[0] - c for c in compositions]
sames = [tfs.is_same_transform(compositions[0], c) for c in compositions]