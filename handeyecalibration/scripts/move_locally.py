
from __future__ import division
import rospy
from abc import ABCMeta, abstractmethod
from tf.transformations import quaternion_multiply, quaternion_matrix, quaternion_from_euler, euler_from_quaternion
from math import pi
import math
import itertools
from std_msgs.msg import String, Header, Empty
from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_commander import MoveGroupCommander
from copy import deepcopy
import numpy as np


def rot_per_joint(plan, degrees=False):
    np_traj = np.array([p.positions for p in plan.joint_trajectory.points])
    if len(np_traj) == 0:
        raise ValueError
    np_traj_max_per_joint = np_traj.max(axis=0)
    np_traj_min_per_joint = np_traj.min(axis=0)
    ret = abs(np_traj_max_per_joint - np_traj_min_per_joint)
    if degrees:
        ret = [math.degrees(j) for j in ret]
    return ret


def is_crazy_plan(plan, max_rotation_per_joint):
    abs_rot_per_joint = rot_per_joint(plan)
    if (abs_rot_per_joint > max_rotation_per_joint).any():
        return True
    else:
        return False


NODE_NAME = 'local_mover'

rospy.init_node(NODE_NAME)
while rospy.get_time() == 0.0: pass

move_group_name = rospy.get_param('move_group')
angle_delta = rospy.get_param('angle_delta')

mgc = MoveGroupCommander(move_group_name)
rospy.sleep(1.0)

c = mgc.get_current_pose()
d = deepcopy(c)

basis = np.eye(3)

quaternion_deltas = [quaternion_from_euler(*rot_axis*angle_delta) for rot_axis in basis]

final_rots = []
for qd in quaternion_deltas:
    final_rots.append(list(qd))
    final_rots.append(list(-qd))

final_poses = []
for rot in final_rots:
    fp = deepcopy(c)
    fp.
    final_poses.append(fp)

# TODO: for each pose by rotating in a direction, plan when a button is pressed,
# execute that plan when other button is pressed

# TODO: alternative workflow for automatic calibration:
# generate poses around current pose, each rotating by angle_delta in each direction
# generate a plan to each pose and back
# if all movements are possible, perform them in a row (at low speed)



