import rospy
import std_srvs
from std_srvs import srv
import easy_handeye as hec
from easy_handeye import srv


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

        rospy.wait_for_service(hec.GET_SAMPLE_LIST_TOPIC)
        self.get_sample_proxy = rospy.ServiceProxy(hec.GET_SAMPLE_LIST_TOPIC, hec.srv.TakeSample)
        rospy.wait_for_service(hec.TAKE_SAMPLE_TOPIC)
        self.take_sample_proxy = rospy.ServiceProxy(hec.TAKE_SAMPLE_TOPIC, hec.srv.TakeSample)
        rospy.wait_for_service(hec.REMOVE_SAMPLE_TOPIC)
        self.remove_sample_proxy = rospy.ServiceProxy(hec.REMOVE_SAMPLE_TOPIC, hec.srv.RemoveSample)
        rospy.wait_for_service(hec.COMPUTE_CALIBRATION_TOPIC)
        self.compute_calibration_proxy = rospy.ServiceProxy(hec.COMPUTE_CALIBRATION_TOPIC, hec.srv.ComputeCalibration)
        rospy.wait_for_service(hec.SAVE_CALIBRATION_TOPIC)
        self.save_calibration_proxy = rospy.ServiceProxy(hec.SAVE_CALIBRATION_TOPIC, std_srvs.srv.Empty)

    def get_sample_list(self):
        return self.get_sample_proxy().samples

    def take_sample(self):
        return self.take_sample_proxy().samples

    def remove_sample(self, index):
        return self.remove_sample_proxy(hec.srv.RemoveSampleRequest(sample_index=index)).samples

    def compute_calibration(self):
        return self.compute_calibration_proxy()

    def save(self):
        return self.save_calibration_proxy()
