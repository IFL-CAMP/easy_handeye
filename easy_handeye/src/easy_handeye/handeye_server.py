import rospy
import std_msgs
import std_srvs
from std_srvs import srv
import easy_handeye as hec
import easy_handeye_msgs as ehm
from easy_handeye_msgs import msg, srv
from easy_handeye.handeye_sampler import HandeyeSampler
from easy_handeye.handeye_calibration import HandeyeCalibrationParameters
from easy_handeye.handeye_calibration_service_visp import HandeyeCalibrationServiceVisp


class HandeyeServer:
    def __init__(self):

        self.parameters = HandeyeCalibrationParameters(namespace=rospy.get_namespace(),
                                                       eye_on_hand=rospy.get_param('eye_on_hand'),
                                                       robot_effector_frame=rospy.get_param('robot_effector_frame'),
                                                       robot_base_frame=rospy.get_param('robot_base_frame'),
                                                       tracking_base_frame=rospy.get_param('tracking_base_frame'),
                                                       tracking_marker_frame=rospy.get_param('tracking_marker_frame'))

        self.sampler = HandeyeSampler(handeye_parameters=self.parameters)
        self.calibration_service = HandeyeCalibrationServiceVisp()

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

        # Useful for secondary input sources (e.g. programmable buttons on robot)
        self.take_sample_topic = rospy.Subscriber(hec.TAKE_SAMPLE_TOPIC,
                                                  std_msgs.msg.Empty, self.take_sample)
        self.compute_calibration_topic = rospy.Subscriber(hec.REMOVE_SAMPLE_TOPIC,
                                                          std_msgs.msg.Empty,
                                                          self.remove_last_sample)

        self.last_calibration = None

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
            self.last_calibration.to_file()
            rospy.loginfo('Calibration saved to {}'.format(self.last_calibration.filename))
        return std_srvs.srv.EmptyResponse()
