#!/usr/bin/env python2
from __future__ import division
import rospy
from tf.transformations import quaternion_multiply, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from moveit_commander import MoveGroupCommander
from copy import deepcopy
import numpy as np
from itertools import chain, izip
import math
from python_qt_binding import QtGui
import sys


class LocalMover:
    def __init__(self, move_group_name):
        self.mgc = MoveGroupCommander(move_group_name)
        self.start_pose = self.mgc.get_current_pose()
        self.poses = []
        self.current_pose_index = -1
        self.fallback_joint_limits = [math.radians(90)]*4+[math.radians(90)]+[math.radians(180)]+[math.radians(350)]
        if len(self.mgc.get_active_joints()) == 6:
            self.fallback_joint_limits = self.fallback_joint_limits[1:]

    def compute_poses_around_current_state(self, angle_delta):
        self.start_pose = self.mgc.get_current_pose()
        basis = np.eye(3)

        pos_deltas = [quaternion_from_euler(*rot_axis*angle_delta) for rot_axis in basis]
        neg_deltas = [quaternion_from_euler(*rot_axis*(-angle_delta)) for rot_axis in basis]

        quaternion_deltas = list(chain.from_iterable(izip(pos_deltas, neg_deltas)))  # interleave

        final_rots = []
        for qd in quaternion_deltas:
            final_rots.append(list(qd))

        final_poses = []
        for rot in final_rots:
            fp = deepcopy(self.start_pose)
            ori = fp.pose.orientation
            combined_rot = quaternion_multiply([ori.x, ori.y, ori.z, ori.w], rot)
            fp.pose.orientation = Quaternion(*combined_rot)
            final_poses.append(fp)

        self.poses = final_poses
        self.current_pose_index = -1

    def check_poses(self, joint_limits):
        if len(self.fallback_joint_limits) == 6:
            joint_limits = joint_limits[1:]
        for fp in self.poses:
            self.mgc.set_pose_target(fp)
            plan = self.mgc.plan()
            if len(plan.joint_trajectory.points) == 0 or LocalMover.is_crazy_plan(plan, joint_limits):
                # TODO: handle failure better?
                raise RuntimeError('Cannot do calibration from this starting pose!')

    def go_to_pose_number(self, index):
        self.go_to_pose(self.poses[index])

    def go_to_next_pose(self):
        if self.current_pose_index >= len(self.poses)-1:
            return False

        self.go_to_start_pose()
        self.current_pose_index += 1
        self.go_to_pose_number(self.current_pose_index)

    def go_to_start_pose(self):
        self.go_to_pose(self.start_pose)

    def go_to_pose(self, pose):
        self.mgc.set_start_state_to_current_state()
        self.mgc.set_pose_target(pose)
        plan = self.mgc.plan()
        if LocalMover.is_crazy_plan(plan, self.fallback_joint_limits):
            raise RuntimeError("got crazy plan!")
        self.mgc.execute(plan)

    @staticmethod
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

    @staticmethod
    def is_crazy_plan(plan, max_rotation_per_joint):
        abs_rot_per_joint = LocalMover.rot_per_joint(plan)
        if (abs_rot_per_joint > max_rotation_per_joint).any():
            return True
        else:
            return False


class LocalMoverGUI(QtGui.QWidget):
    def __init__(self, local_mover):
        super(LocalMoverGUI, self).__init__()
        self.local_mover = local_mover
        self.initUI()

    def initUI(self):
        self.layout = QtGui.QVBoxLayout()
        self.labels_layout = QtGui.QHBoxLayout()
        self.buttons_layout = QtGui.QHBoxLayout()

        self.pose_number_lbl = QtGui.QLabel('0/6')
        self.bad_plan_lbl = QtGui.QLabel('No plan yet')

        self.next_pose_btn = QtGui.QPushButton('Next Pose')
        self.next_pose_btn.clicked.connect(self.local_mover.go_to_next_pose)

        self.plan_btn = QtGui.QPushButton('Plan')
        self.plan_btn.clicked.connect(self.local_mover.go_to_next_pose)

        self.execute_btn = QtGui.QPushButton('Execute')
        self.execute_btn.clicked.connect(self.local_mover.go_to_next_pose)

        self.labels_layout.addWidget(self.pose_number_lbl)
        self.labels_layout.addWidget(self.bad_plan_lbl)

        self.buttons_layout.addWidget(self.next_pose_btn)
        self.buttons_layout.addWidget(self.plan_btn)
        self.buttons_layout.addWidget(self.execute_btn)

        self.layout.addLayout(self.labels_layout)
        self.layout.addLayout(self.buttons_layout)

        self.setLayout(self.layout)

        self.setWindowTitle('Local Mover')
        self.show()


NODE_NAME = 'local_mover'

rospy.init_node(NODE_NAME)
while rospy.get_time() == 0.0: pass

move_group_name = rospy.get_param('~move_group', 'manipulator')
angle_delta = rospy.get_param('~angle_delta', math.radians(25))

lm = LocalMover(move_group_name)
rospy.sleep(1.0)

lm.compute_poses_around_current_state(angle_delta)

joint_limits = [math.radians(25)]*4+[math.radians(90)]+[math.radians(180)]+[math.radians(350)]  # TODO: make param
lm.check_poses(joint_limits)

qapp = QtGui.QApplication(sys.argv)
lmg = LocalMoverGUI(lm)
lmg.show()
sys.exit(qapp.exec_())

# TODO: alternative workflow for automatic calibration:
# generate poses around current pose, each rotating by angle_delta in each direction
# generate a plan to each pose and back
# if all movements are possible, perform them in a row (at low speed)



