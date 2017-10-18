import os
import yaml
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped


class HandeyeCalibration(object):
    """
    Stores parameters and transformation of a hand-eye calibration for publishing.
    """
    DIRECTORY = os.path.expanduser('~/.ros/easy_handeye')
    """Default directory for calibration yaml files."""

    # TODO: use the HandeyeCalibration message instead, this should be HandeyeCalibrationConversions
    def __init__(self,
                 eye_on_hand=False,
                 robot_base_frame=None,
                 robot_effector_frame=None,
                 tracking_base_frame=None,
                 transformation=None):
        """
        Creates a HandeyeCalibration object.

        :param eye_on_hand: if false, it is a eye-on-base calibration
        :type eye_on_hand: bool
        :param robot_base_frame: needed only for eye-on-base calibrations: robot base link tf name
        :type robot_base_frame: string
        :param robot_effector_frame: needed only for eye-on-hand calibrations: robot tool tf name
        :type robot_effector_frame: string
        :param tracking_base_frame: tracking system tf name
        :type tracking_base_frame: string
        :param transformation: transformation between optical origin and base/tool robot frame as tf tuple
        :type transformation: ((float, float, float), (float, float, float, float))
        :return: a HandeyeCalibration object

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """

        if transformation is None:
            transformation = ((0, 0, 0), (0, 0, 0, 1))

        self.eye_on_hand = eye_on_hand
        """
        if false, it is a eye-on-base calibration

        :type: bool
        """

        self.transformation = TransformStamped(transform=Transform(
            Vector3(*transformation[0]), Quaternion(*transformation[1])))
        """
        transformation between optical origin and base/tool robot frame

        :type: geometry_msgs.msg.TransformedStamped
        """

        # tf names
        if self.eye_on_hand:
            self.transformation.header.frame_id = robot_effector_frame
        else:
            self.transformation.header.frame_id = robot_base_frame
        self.transformation.child_frame_id = tracking_base_frame

        self.filename = HandeyeCalibration.DIRECTORY + '/' + rospy.get_namespace().rstrip('/').split('/')[-1] + '.yaml'

    def to_dict(self):
        """
        Returns a dictionary representing this calibration.

        :return: a dictionary representing this calibration.

        :rtype: dict[string, string|dict[string,float]]
        """
        ret = {
            'eye_on_hand': self.eye_on_hand,
            'tracking_base_frame': self.transformation.child_frame_id,
            'transformation': {
                'x': self.transformation.transform.translation.x,
                'y': self.transformation.transform.translation.y,
                'z': self.transformation.transform.translation.z,
                'qx': self.transformation.transform.rotation.x,
                'qy': self.transformation.transform.rotation.y,
                'qz': self.transformation.transform.rotation.z,
                'qw': self.transformation.transform.rotation.w
            }
        }
        if self.eye_on_hand:
            ret['robot_effector_frame'] = self.transformation.header.frame_id
        else:
            ret['robot_base_frame'] = self.transformation.header.frame_id

        return ret

    def from_dict(self, in_dict):
        """
        Sets values parsed from a given dictionary.

        :param in_dict: input dictionary.
        :type in_dict: dict[string, string|dict[string,float]]

        :rtype: None
        """
        self.eye_on_hand = in_dict['eye_on_hand']
        self.transformation = TransformStamped(
            child_frame_id=in_dict['tracking_base_frame'],
            transform=Transform(
                Vector3(in_dict['transformation']['x'],
                        in_dict['transformation']['y'],
                        in_dict['transformation']['z']),
                Quaternion(in_dict['transformation']['qx'],
                           in_dict['transformation']['qy'],
                           in_dict['transformation']['qz'],
                           in_dict['transformation']['qw'])
            )
        )
        if self.eye_on_hand:
            self.transformation.header.frame_id = in_dict['robot_effector_frame']
        else:
            self.transformation.header.frame_id = in_dict['robot_base_frame']

    def to_yaml(self):
        """
        Returns a yaml string representing this calibration.

        :return: a yaml string

        :rtype: string
        """
        return yaml.dump(self.to_dict())

    def from_yaml(self, in_yaml):
        """
        Parses a yaml string and sets the contained values in this calibration.

        :param in_yaml: a yaml string
        :rtype: None
        """
        self.from_dict(yaml.load(in_yaml))

    def to_file(self):
        """
        Saves this calibration in a yaml file in the default path.

        The default path consists of the default directory and the namespace the node is running in.

        :rtype: None
        """
        if not os.path.exists(HandeyeCalibration.DIRECTORY):
            os.makedirs(HandeyeCalibration.DIRECTORY)

        with open(self.filename, 'w') as calib_file:
            calib_file.write(self.to_yaml())

    def from_file(self):
        """
        Parses a yaml file in the default path and sets the contained values in this calibration.

        The default path consists of the default directory and the namespace the node is running in.

        :rtype: None
        """

        with open(self.filename) as calib_file:
            self.from_yaml(calib_file.read())

    def from_parameters(self):
        """
        Stores this calibration as ROS parameters in the namespace of the current node.

        :rtype: None
        """
        calib_dict = {}

        root_params = ['eye_on_hand', 'tracking_base_frame']
        for rp in root_params:
            calib_dict[rp] = rospy.get_param(rp)

        if calib_dict['eye_on_hand']:
            calib_dict['robot_effector_frame'] = rospy.get_param('robot_effector_frame')
        else:
            calib_dict['robot_base_frame'] = rospy.get_param('robot_base_frame')

        transf_params = 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
        calib_dict['transformation'] = {}
        for tp in transf_params:
            calib_dict['transformation'][tp] = rospy.get_param('transformation/'+tp)

        self.from_dict(calib_dict)

    def to_parameters(self):
        """
        Fetches a calibration from ROS parameters in the namespace of the current node into this object.

        :rtype: None
        """
        calib_dict = self.to_dict()

        root_params = ['eye_on_hand', 'tracking_base_frame']
        if calib_dict['eye_on_hand']:
            root_params.append('robot_effector_frame')
        else:
            root_params.append('robot_base_frame')

        for rp in root_params:
            rospy.set_param(rp, calib_dict[rp])

        transf_params = 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'

        for tp in transf_params:
            rospy.set_param('transformation/'+tp, calib_dict['transformation'][tp])

