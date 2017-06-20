import rospy
import std_msgs
import std_srvs
from std_srvs import srv
import easy_handeye as hec
from easy_handeye import srv
from easy_handeye.msg import SampleList
from easy_handeye.handeye_calibrator import HandeyeCalibrator


class HandeyeServer:
    def __init__(self):
        self.calibrator = HandeyeCalibrator()

        self.get_sample_list_service = rospy.Service(hec.GET_SAMPLE_LIST_TOPIC,
                                                     hec.srv.TakeSample, self.get_sample_lists)
        self.take_sample_service = rospy.Service(hec.TAKE_SAMPLE_TOPIC,
                                                 hec.srv.TakeSample, self.take_sample)
        self.remove_sample_service = rospy.Service(hec.REMOVE_SAMPLE_TOPIC,
                                                   hec.srv.RemoveSample, self.remove_sample)
        self.compute_calibration_service = rospy.Service(hec.COMPUTE_CALIBRATION_TOPIC,
                                                         hec.srv.ComputeCalibration, self.compute_calibration)
        self.save_calibration_service = rospy.Service(hec.SAVE_CALIBRATION_TOPIC,
                                                      std_srvs.srv.Empty, self.save_calibration)

        # Useful for secondary input sources (e.g. programmable buttons on robot)
        self.take_sample_topic = rospy.Subscriber(hec.TAKE_SAMPLE_TOPIC,
                                                  std_msgs.msg.Empty, self.take_sample)
        self.compute_calibration_topic = rospy.Subscriber(hec.REMOVE_SAMPLE_TOPIC,
                                                          std_msgs.msg.Empty, self.remove_last_sample)  # TODO: normalize

        self.last_calibration = None

    def get_sample_lists(self, _):
        return hec.srv.TakeSampleResponse(SampleList(*self.calibrator.get_visp_samples()))

    def take_sample(self, _):
        self.calibrator.take_sample()
        return hec.srv.TakeSampleResponse(SampleList(*self.calibrator.get_visp_samples()))

    def remove_last_sample(self):
        self.calibrator.remove_sample(len(self.calibrator.samples)-1)

    def remove_sample(self, req):
        try:
            self.calibrator.remove_sample(req.sample_index)
        except IndexError:
            rospy.logerr('Invalid index ' + req.sample_index)
        return hec.srv.RemoveSampleResponse(SampleList(*self.calibrator.get_visp_samples()))

    def compute_calibration(self, _):
        self.last_calibration = self.calibrator.compute_calibration()
        # TODO: avoid confusion class/msg, change class into HandeyeCalibrationConversions
        ret = hec.srv.ComputeCalibrationResponse()
        if self.last_calibration is None:
            rospy.logwarn('No valid calibration computed, returning null')
            return ret
        ret.calibration.eye_on_hand = self.last_calibration.eye_on_hand
        ret.calibration.transform = self.last_calibration.transformation
        return ret

    def save_calibration(self, _):
        self.last_calibration.to_parameters()
        self.last_calibration.to_file()
        return std_srvs.srv.EmptyResponse()
