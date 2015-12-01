#!/usr/bin/env python

import os
import yaml
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
import handeye_calibration as hec


class HandEyeCalibratorWrapper:
    def __init__(self):
        self.calibrator = hec.HandeyeCalibrator()

        self.take_sample_subscriber = rospy.Subscriber(hec.TAKE_SAMPLE_TOPIC, Empty, self.take_sample)
        self.remove_sample_subscriber = rospy.Subscriber(hec.REMOVE_SAMPLE_TOPIC, Empty, self.remove_sample)
        self.compute_calibration_subscriber = rospy.Subscriber(hec.COMPUTE_CALIBRATION_TOPIC, Empty, self.compute_calibration)
        self.save_calibration_subscriber = rospy.Subscriber(hec.SAVE_CALIBRATION_TOPIC, Empty, self.save_calibration)

        self.hand_world_sample_list_publisher = rospy.Publisher(hec.HAND_WORLD_SAMPLE_LIST_TOPIC, TransformArray)
        self.camera_marker_sample_list_publisher = rospy.Publisher(hec.CAMERA_MARKER_SAMPLE_LIST_TOPIC, TransformArray)
        self.calibration_result_publisher = rospy.Publisher(hec.CALIBRATION_RESULT_TOPIC, Transform)

    def _publish_sample_lists(self):
        self.calibrator._inner_to_visp_samples()
        self.hand_world_sample_list_publisher.publish(self.calibrator.hand_world_samples)
        self.camera_marker_sample_list_publisher.publish(self.calibrator.camera_marker_samples)

    def take_sample(self):
        self.calibrator.take_sample()
        self._publish_sample_lists()

    def remove_sample(self, index):
        try:
            self.calibrator.remove_sample(index)
        except IndexError:
            rospy.logerr('Invalid index '+index)
        self._publish_sample_lists()

    def compute_calibration(self):
        base_to_camera = self.calibrator.compute_calibration()
        self.calibration_result_publisher.publish(base_to_camera)

    def save_calibration(self):
        calibration = self.calibrator.compute_calibration()
        self.calibrator.save(calibration)


def main():
    rospy.init_node('handeye_calibration')
    while rospy.get_time() == 0.0:
        pass

    cw = HandEyeCalibratorWrapper()

    rospy.spin()

if __name__ == '__main__':
    main()
