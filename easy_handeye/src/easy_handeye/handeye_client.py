import rospy
import std_srvs
from std_srvs import srv
import easy_handeye as hec
import easy_handeye_msgs as ehm
from easy_handeye_msgs import srv


class HandeyeClient(object):

    def __init__(self):
        self.eye_on_hand = rospy.get_param('eye_on_hand', False)

        # tf names
        self.robot_base_frame = rospy.get_param('robot_base_frame', 'base_link')
        self.robot_effector_frame = rospy.get_param('robot_effector_frame', 'tool0')
        self.tracking_base_frame = rospy.get_param('tracking_base_frame', 'optical_origin')
        self.tracking_marker_frame = rospy.get_param('tracking_marker_frame', 'optical_target')

        rospy.wait_for_service(hec.GET_SAMPLE_LIST_TOPIC)
        self.get_sample_proxy = rospy.ServiceProxy(hec.GET_SAMPLE_LIST_TOPIC, ehm.srv.TakeSample)
        rospy.wait_for_service(hec.TAKE_SAMPLE_TOPIC)
        self.take_sample_proxy = rospy.ServiceProxy(hec.TAKE_SAMPLE_TOPIC, ehm.srv.TakeSample)
        rospy.wait_for_service(hec.REMOVE_SAMPLE_TOPIC)
        self.remove_sample_proxy = rospy.ServiceProxy(hec.REMOVE_SAMPLE_TOPIC, ehm.srv.RemoveSample)
        rospy.wait_for_service(hec.COMPUTE_CALIBRATION_TOPIC)
        self.compute_calibration_proxy = rospy.ServiceProxy(hec.COMPUTE_CALIBRATION_TOPIC, ehm.srv.ComputeCalibration)
        rospy.wait_for_service(hec.SAVE_CALIBRATION_TOPIC)
        self.save_calibration_proxy = rospy.ServiceProxy(hec.SAVE_CALIBRATION_TOPIC, std_srvs.srv.Empty)

    def get_sample_list(self):
        return self.get_sample_proxy().samples

    def take_sample(self):
        return self.take_sample_proxy().samples

    def remove_sample(self, index):
        return self.remove_sample_proxy(ehm.srv.RemoveSampleRequest(sample_index=index)).samples

    def compute_calibration(self):
        return self.compute_calibration_proxy()

    def save(self):
        return self.save_calibration_proxy()
