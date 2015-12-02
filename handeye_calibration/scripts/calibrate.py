#!/usr/bin/env python

import os
import yaml
import rospy
import std_srvs
from geometry_msgs.msg import TransformStamped
from visp_hand2eye_calibration.msg import TransformArray
import handeye_calibration as hec


class HandEyeCalibratorWrapper:
    def __init__(self):
        self.calibrator = hec.HandeyeCalibrator()

        self.take_sample_service = rospy.Service(hec.TAKE_SAMPLE_TOPIC,
                                                 hec.srv.TakeSample, self.take_sample)
        self.remove_sample_service = rospy.Service(hec.REMOVE_SAMPLE_TOPIC,
                                                   hec.srv.RemoveSample, self.remove_sample)
        self.compute_calibration_service = rospy.Service(hec.COMPUTE_CALIBRATION_TOPIC,
                                                         hec.srv.ComputeCalibration, self.compute_calibration)
        self.save_calibration_service = rospy.Service(hec.SAVE_CALIBRATION_TOPIC,
                                                      std_srvs.srv.Empty, self.save_calibration)

        self.calibration_result_publisher = rospy.Publisher(hec.CALIBRATION_RESULT_TOPIC, TransformStamped)

        self.last_calibration = None

    def get_sample_lists(self):
        hand_world_samples, camera_marker_samples = self.calibrator.get_visp_samples()
        return hand_world_samples, camera_marker_samples

    def take_sample(self):
        self.calibrator.take_sample()
        return hec.srv.TakeSampleResponse(*self.get_sample_lists())

    def remove_sample(self, req):
        try:
            self.calibrator.remove_sample(req.sample_index)
        except IndexError:
            rospy.logerr('Invalid index '+req.sample_index)
        return hec.srv.RemoveSampleResponse(*self.get_sample_lists())

    def compute_calibration(self, req):
        self.last_calibration = self.calibrator.compute_calibration()
        return hec.srv.ComputeCalibrationResponse(self.last_calibration)

    def save_calibration(self, req):
        self.last_calibration.to_param()
        self.last_calibration.to_file()
        return std_srvs.srv.EmptyResponse()


def main():
    rospy.init_node('handeye_calibration')
    while rospy.get_time() == 0.0:
        pass

    cw = HandEyeCalibratorWrapper()

    rospy.spin()

if __name__ == '__main__':
    main()
