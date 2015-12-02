import rospy
import std_srvs
from geometry_msgs.msg import TransformStamped

from handeye_calibration import HandeyeCalibration as Hec

class HandeyeServer:
    def __init__(self):
        self.calibrator = Hec.HandeyeCalibrator()

        self.take_sample_service = rospy.Service(Hec.TAKE_SAMPLE_TOPIC,
                                                 Hec.srv.TakeSample, self.take_sample)
        self.remove_sample_service = rospy.Service(Hec.REMOVE_SAMPLE_TOPIC,
                                                   Hec.srv.RemoveSample, self.remove_sample)
        self.compute_calibration_service = rospy.Service(Hec.COMPUTE_CALIBRATION_TOPIC,
                                                         Hec.srv.ComputeCalibration, self.compute_calibration)
        self.save_calibration_service = rospy.Service(Hec.SAVE_CALIBRATION_TOPIC,
                                                      std_srvs.srv.Empty, self.save_calibration)

        self.calibration_result_publisher = rospy.Publisher(Hec.CALIBRATION_RESULT_TOPIC, TransformStamped)

        self.last_calibration = None

    def get_sample_lists(self):
        hand_world_samples, camera_marker_samples = self.calibrator.get_visp_samples()
        return hand_world_samples, camera_marker_samples

    def take_sample(self):
        self.calibrator.take_sample()
        return Hec.srv.TakeSampleResponse(*self.get_sample_lists())

    def remove_sample(self, req):
        try:
            self.calibrator.remove_sample(req.sample_index)
        except IndexError:
            rospy.logerr('Invalid index '+req.sample_index)
        return Hec.srv.RemoveSampleResponse(*self.get_sample_lists())

    def compute_calibration(self, req):
        self.last_calibration = self.calibrator.compute_calibration()
        return Hec.srv.ComputeCalibrationResponse(self.last_calibration)

    def save_calibration(self, req):
        self.last_calibration.to_param()
        self.last_calibration.to_file()
        return std_srvs.srv.EmptyResponse()