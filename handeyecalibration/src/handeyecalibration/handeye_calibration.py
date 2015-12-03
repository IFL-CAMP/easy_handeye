import os
import yaml
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped


class HandeyeCalibration(object):
    """
    Stores parameters and transformation of a hand-eye calibration for publishing.
    """
    DIRECTORY = os.path.expanduser('~/.ros/handeyecalibration')
    """Default directory for calibration yaml files."""

    def __init__(self,
                 eye_on_hand=False,
                 base_link_frame=None,
                 tool_frame=None,
                 optical_origin_frame=None,
                 transformation=None):
        """
        Creates a HandeyeCalibration object.

        :param eye_on_hand: if false, it is a eye-on-base calibration
        :type eye_on_hand: bool
        :param base_link_frame: needed only for eye-on-base calibrations: robot base link tf name
        :type base_link_frame: string
        :param tool_frame: needed only for eye-on-hand calibrations: robot tool tf name
        :type tool_frame: string
        :param optical_origin_frame: tracking system tf name
        :type optical_origin_frame: string
        :param transformation: transformation between optical origin and base/tool robot frame as tf tuple
        :type transformation: ((float, float, float), (float, float, float, float))
        :return: a HandeyeCalibration object

        :rtype: handeyecalibration.handeye_calibration.HandeyeCalibration
        """

        if transformation is None:
            transformation = ((0, 0, 0), (0, 0, 0, 1))

        self.eye_on_hand = eye_on_hand

        self.transformation = TransformStamped(transform=Transform(
            Vector3(*transformation[0]), Quaternion(*transformation[1])))

        # tf names
        if self.eye_on_hand:
            self.transformation.header.frame_id = tool_frame
        else:
            self.transformation.header.frame_id = base_link_frame
        self.transformation.child_frame_id = optical_origin_frame

    def to_dict(self):
        """
        Returns a dictionary representing this calibration.

        :return: a dictionary representing this calibration.

        :rtype: dict[string, string|dict[string,float]]
        """
        ret = {
            'eye_on_hand': self.eye_on_hand,
            'optical_origin_frame': self.transformation.child_frame_id,
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
            ret['tool_frame'] = self.transformation.header.frame_id
        else:
            ret['base_link_frame'] = self.transformation.header.frame_id

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
            child_frame_id=in_dict['optical_origin_frame'],
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
            self.transformation.header.frame_id = in_dict['tool_frame']
        else:
            self.transformation.header.frame_id = in_dict['base_link_frame']

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
        filename = HandeyeCalibration.DIRECTORY + rospy.get_namespace() + '.yaml'

        with open(filename, 'w') as calib_file:
            calib_file.write(self.to_yaml())

    def from_file(self):
        """
        Parses a yaml file in the default path and sets the contained values in this calibration.

        The default path consists of the default directory and the namespace the node is running in.

        :rtype: None
        """
        filename = HandeyeCalibration.DIRECTORY + rospy.get_namespace() + '.yaml'

        with open(filename) as calib_file:
            self.from_yaml(calib_file.read())

    def from_parameters(self):
        """
        Stores this calibration as ROS parameters in the namespace of the current node.

        :rtype: None
        """
        calib_dict = {}

        root_params = ['eye_on_hand', 'optical_origin_frame']
        for rp in root_params:
            calib_dict[rp] = rospy.get_param(rp)

        if calib_dict['eye_on_hand']:
            calib_dict['tool_frame'] = rospy.get_param('tool_frame')
        else:
            calib_dict['base_link_frame'] = rospy.get_param('base_link_frame')

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

        root_params = ['eye_on_hand', 'optical_origin_frame']
        if calib_dict['eye_on_hand']:
            root_params.append('tool_frame')
        else:
            root_params.append('base_link_frame')

        for rp in root_params:
            rospy.set_param(rp, calib_dict[rp])

        transf_params = 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'

        for tp in transf_params:
            rospy.set_param('transformation/'+tp, calib_dict['transformation'][tp])

