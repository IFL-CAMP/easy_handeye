import rospy
import std_srvs
from std_srvs import srv
import handeyecalibration as hec
from handeyecalibration import srv

class HandeyeClient(object):

    def __init__(self):
        self.eye_on_hand = rospy.get_param('eye_on_hand', False)

        # tf names
        if self.eye_on_hand:
            self.tool_frame = rospy.get_param('tool_frame', 'tool0')
            self.base_link_frame = None
        else:
            self.tool_frame = None
            self.base_link_frame = rospy.get_param('base_link_frame', 'base_link')

        self.optical_origin_frame = rospy.get_param('optical_origin_frame', 'optical_origin')
        self.optical_target_frame = rospy.get_param('optical_target_frame', 'optical_target')

        self.get_sample_proxy = rospy.ServiceProxy(hec.GET_SAMPLE_LIST_TOPIC, hec.srv.TakeSample)
        self.take_sample_proxy = rospy.ServiceProxy(hec.TAKE_SAMPLE_TOPIC, hec.srv.TakeSample)
        self.remove_sample_proxy = rospy.ServiceProxy(hec.REMOVE_SAMPLE_TOPIC, hec.srv.RemoveSample)
        self.compute_calibration_proxy = rospy.ServiceProxy(hec.COMPUTE_CALIBRATION_TOPIC, hec.srv.ComputeCalibration)
        self.save_calibration_proxy = rospy.ServiceProxy(hec.SAVE_CALIBRATION_TOPIC, std_srvs.srv.Empty)

    def get_sample_list(self):
        return self.get_sample_proxy()

    def take_sample(self):
        return self.take_sample_proxy()

    def remove_sample(self, index):
        return self.remove_sample_proxy(index)

    def compute_calibration(self):
        return self.compute_calibration_proxy()

    def save(self):
        return self.save_calibration_proxy()
