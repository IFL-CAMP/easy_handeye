import math

import easy_handeye_msgs as ehm
import rospy
import std_msgs
import std_srvs
from easy_handeye_msgs import msg, srv
from std_msgs import msg
from std_srvs import srv

import easy_handeye as hec
from easy_handeye.handeye_calibration import HandeyeCalibration, HandeyeCalibrationParameters
from easy_handeye.handeye_calibration_service_visp import HandeyeCalibrationServiceVisp
from easy_handeye.handeye_robot import CalibrationMovements
from easy_handeye.handeye_sampler import HandeyeSampler


class HandeyeServer:
    def __init__(self, namespace=None):
        if namespace is None:
            namespace = rospy.get_namespace()
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

        self.sampler = HandeyeSampler(handeye_parameters=self.parameters)
        self.calibration_service = HandeyeCalibrationServiceVisp()

        # setup calibration services and topics

        self.get_sample_list_service = rospy.Service(hec.GET_SAMPLE_LIST_TOPIC,
                                                     ehm.srv.TakeSample, self.get_sample_lists)
        self.take_sample_service = rospy.Service(hec.TAKE_SAMPLE_TOPIC,
                                                 ehm.srv.TakeSample, self.take_sample)
        self.remove_sample_service = rospy.Service(hec.REMOVE_SAMPLE_TOPIC,
                                                   ehm.srv.RemoveSample, self.remove_sample)
        self.compute_calibration_service = rospy.Service(hec.COMPUTE_CALIBRATION_TOPIC,
                                                         ehm.srv.ComputeCalibration, self.compute_calibration)
        self.save_calibration_service = rospy.Service(hec.SAVE_CALIBRATION_TOPIC,
                                                      std_srvs.srv.Empty, self.save_calibration)

        self.check_starting_position_service = rospy.Service(hec.CHECK_STARTING_POSE_TOPIC, ehm.srv.CheckStartingPose,
                                                             self.check_starting_position)
        self.enumerate_target_poses_service = rospy.Service(hec.ENUMERATE_TARGET_POSES_TOPIC,
                                                            ehm.srv.EnumerateTargetPoses, self.enumerate_target_poses)
        self.select_target_pose_service = rospy.Service(hec.SELECT_TARGET_POSE_TOPIC, ehm.srv.SelectTargetPose,
                                                        self.select_target_pose)
        self.plan_to_selected_target_pose_service = rospy.Service(hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC,
                                                                  ehm.srv.PlanToSelectedTargetPose,
                                                                  self.plan_to_selected_target_pose)
        self.execute_plan_service = rospy.Service(hec.EXECUTE_PLAN_TOPIC, ehm.srv.ExecutePlan, self.execute_plan)

        # Useful for secondary input sources (e.g. programmable buttons on robot)
        self.take_sample_topic = rospy.Subscriber(hec.TAKE_SAMPLE_TOPIC,
                                                  std_msgs.msg.Empty, self.take_sample)
        self.compute_calibration_topic = rospy.Subscriber(hec.REMOVE_SAMPLE_TOPIC,
                                                          std_msgs.msg.Empty,
                                                          self.remove_last_sample)

        self.last_calibration = None

    # sampling

    def _retrieve_sample_list(self):
        ret = ehm.msg.SampleList()
        for s in self.sampler.get_samples():
            ret.camera_marker_samples.append(s['optical'].transform)
            ret.hand_world_samples.append(s['robot'].transform)
        return ret

    def get_sample_lists(self, _):
        return ehm.srv.TakeSampleResponse(self._retrieve_sample_list())

    def take_sample(self, _):
        self.sampler.take_sample()
        return ehm.srv.TakeSampleResponse(self._retrieve_sample_list())

    def remove_last_sample(self):
        self.sampler.remove_sample(len(self.sampler.samples) - 1)

    def remove_sample(self, req):
        try:
            self.sampler.remove_sample(req.sample_index)
        except IndexError:
            rospy.logerr('Invalid index ' + req.sample_index)
        return ehm.srv.RemoveSampleResponse(self._retrieve_sample_list())

    # calibration

    def compute_calibration(self, _):
        samples = self.sampler.get_samples()
        self.last_calibration = self.calibration_service.compute_calibration(self.parameters, samples)
        ret = ehm.srv.ComputeCalibrationResponse()
        if self.last_calibration is None:
            rospy.logwarn('No valid calibration computed')
            ret.valid = False
            return ret
        ret.valid = True
        ret.calibration.eye_on_hand = self.last_calibration.parameters.eye_on_hand
        ret.calibration.transform = self.last_calibration.transformation
        return ret

    def save_calibration(self, _):
        if self.last_calibration:
            HandeyeCalibration.to_file(self.last_calibration)
            rospy.loginfo('Calibration saved to {}'.format(self.last_calibration.filename()))
        return std_srvs.srv.EmptyResponse()

    # TODO: evaluation

    # robot movement

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
