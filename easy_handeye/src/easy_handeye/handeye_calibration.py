import os
import yaml
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped


# TODO: make this a data class in python3
class HandeyeCalibrationParameters(object):
    def __init__(self, namespace, eye_on_hand=None, robot_base_frame=None, robot_effector_frame=None,
                 tracking_base_frame=None,
                 tracking_marker_frame=None):
        """
        Creates a HandeyeCalibrationParameters object.

        :param namespace: the namespace of the calibration (will determine the filename)
        :type namespace: string
        :param eye_on_hand: if false, it is a eye-on-base calibration
        :type eye_on_hand: bool
        :param robot_base_frame: needed only for eye-on-base calibrations: robot base link tf name
        :type robot_base_frame: string
        :param robot_effector_frame: needed only for eye-on-hand calibrations: robot tool tf name
        :type robot_effector_frame: string
        :param tracking_base_frame: tracking system tf name
        :type tracking_base_frame: string
        """
        self.namespace = namespace
        self.eye_on_hand = eye_on_hand
        self.robot_base_frame = robot_base_frame
        self.robot_effector_frame = robot_effector_frame
        self.tracking_base_frame = tracking_base_frame
        self.tracking_marker_frame = tracking_marker_frame


class HandeyeCalibration(object):
    """
    Stores parameters and transformation of a hand-eye calibration for publishing.
    """
    DIRECTORY = os.path.expanduser('~/.ros/easy_handeye')
    """Default directory for calibration yaml files."""

    # TODO: use the HandeyeCalibration message instead, this should be HandeyeCalibrationConversions
    def __init__(self,
                 handeye_parameters=None,
                 transformation=None):
        """
        Creates a HandeyeCalibration object.

        :param transformation: transformation between optical origin and base/tool robot frame as tf tuple
        :type transformation: ((float, float, float), (float, float, float, float))
        :return: a HandeyeCalibration object

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        self.parameters = handeye_parameters

        if transformation is None:
            transformation = ((0, 0, 0), (0, 0, 0, 1))

        self.transformation = TransformStamped(transform=Transform(
            Vector3(*transformation[0]), Quaternion(*transformation[1])))
        """
        transformation between optical origin and base/tool robot frame

        :type: geometry_msgs.msg.TransformedStamped
        """

        # tf names
        if self.parameters.eye_on_hand:
            self.transformation.header.frame_id = handeye_parameters.robot_effector_frame
        else:
            self.transformation.header.frame_id = handeye_parameters.robot_base_frame
        self.transformation.child_frame_id = handeye_parameters.tracking_base_frame

        self.filename = HandeyeCalibration.DIRECTORY + '/' + handeye_parameters.namespace.rstrip('/').split('/')[
            -1] + '.yaml'

    def to_dict(self):
        """
        Returns a dictionary representing this calibration.

        :return: a dictionary representing this calibration.

        :rtype: dict[string, string|dict[string,float]]
        """
        ret = {
            'eye_on_hand': self.parameters.eye_on_hand,
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
        if self.parameters.eye_on_hand:
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
        self.parameters.eye_on_hand = in_dict['eye_on_hand']
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
        if self.parameters.eye_on_hand:
            self.transformation.header.frame_id = in_dict['robot_effector_frame']
        else:
            self.transformation.header.frame_id = in_dict['robot_base_frame']

    def to_yaml(self):
        """
        Returns a yaml string representing this calibration.

        :return: a yaml string

        :rtype: string
        """
        return yaml.dump(self.to_dict(), default_flow_style=False)

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
