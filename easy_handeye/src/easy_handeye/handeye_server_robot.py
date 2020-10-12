import math

import easy_handeye_msgs as ehm
import rospy
from easy_handeye_msgs import msg, srv

import easy_handeye as hec
from easy_handeye.handeye_calibration import HandeyeCalibrationParameters
from easy_handeye.handeye_robot import CalibrationMovements


class HandeyeServerRobot:
    def __init__(self, namespace=None):
        if namespace is None:
            namespace = rospy.get_namespace()
        if not namespace.endswith('/'):
            namespace = namespace+'/'
        self.parameters = HandeyeCalibrationParameters.init_from_parameter_server(namespace)
        angle_delta = math.radians(rospy.get_param('~rotation_delta_degrees', 25))
        translation_delta = rospy.get_param('~translation_delta_meters', 0.1)
        max_velocity_scaling = rospy.get_param('~max_velocity_scaling', 0.5)
        max_acceleration_scaling = rospy.get_param('~max_acceleration_scaling', 0.5)
        self.local_mover = CalibrationMovements(move_group_name=self.parameters.move_group,
                                                move_group_namespace=self.parameters.move_group_namespace,
                                                max_velocity_scaling=max_velocity_scaling,
                                                max_acceleration_scaling=max_acceleration_scaling,
                                                angle_delta=angle_delta, translation_delta=translation_delta)

        # setup services and topics

        self.check_starting_position_service = rospy.Service(namespace+hec.CHECK_STARTING_POSE_TOPIC, ehm.srv.CheckStartingPose,
                                                             self.check_starting_position)
        self.enumerate_target_poses_service = rospy.Service(namespace+hec.ENUMERATE_TARGET_POSES_TOPIC,
                                                            ehm.srv.EnumerateTargetPoses, self.enumerate_target_poses)
        self.select_target_pose_service = rospy.Service(namespace+hec.SELECT_TARGET_POSE_TOPIC, ehm.srv.SelectTargetPose,
                                                        self.select_target_pose)
        self.plan_to_selected_target_pose_service = rospy.Service(namespace+hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC,
                                                                  ehm.srv.PlanToSelectedTargetPose,
                                                                  self.plan_to_selected_target_pose)
        self.execute_plan_service = rospy.Service(namespace+hec.EXECUTE_PLAN_TOPIC, ehm.srv.ExecutePlan, self.execute_plan)

    def check_starting_position(self, _):
        can_calibrate = self.local_mover.set_and_check_starting_position()
        target_poses = ehm.msg.TargetPoseList(home_pose=self.local_mover.start_pose,
                                              target_poses=self.local_mover.target_poses,
                                              current_target_pose_index=self.local_mover.current_pose_index)
        return ehm.srv.CheckStartingPoseResponse(can_calibrate=can_calibrate, target_poses=target_poses)

    def enumerate_target_poses(self, _):
        target_poses = ehm.msg.TargetPoseList(home_pose=self.local_mover.start_pose,
                                              target_poses=self.local_mover.target_poses,
                                              current_target_pose_index=self.local_mover.current_pose_index)
        return ehm.srv.EnumerateTargetPosesResponse(target_poses=target_poses)

    def select_target_pose(self, req):
        success = self.local_mover.select_target_pose(req.target_pose_index)
        target_poses = ehm.msg.TargetPoseList(home_pose=self.local_mover.start_pose,
                                              target_poses=self.local_mover.target_poses,
                                              current_target_pose_index=self.local_mover.current_pose_index)
        return ehm.srv.SelectTargetPoseResponse(success=success, target_poses=target_poses)

    def plan_to_selected_target_pose(self, _):
        ret = self.local_mover.plan_to_current_target_pose()
        return ehm.srv.PlanToSelectedTargetPoseResponse(success=ret)

    def execute_plan(self, _):
        ret = self.local_mover.execute_plan()
        return ehm.srv.ExecutePlanResponse(success=ret)
