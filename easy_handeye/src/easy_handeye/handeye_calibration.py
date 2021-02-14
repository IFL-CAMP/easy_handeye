import os
import yaml

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped


# TODO: make this a data class in python3
class HandeyeCalibrationParameters(object):
    def __init__(self, namespace, move_group_namespace='/', move_group='manipulator', eye_on_hand=None,
                 robot_base_frame=None, robot_effector_frame=None,
                 tracking_base_frame=None,
                 tracking_marker_frame=None,
                 freehand_robot_movement=None):
        """
        Creates a HandeyeCalibrationParameters object.

        :param namespace: the namespace of the calibration (will determine the filename)
        :type namespace: string
        :param move_group: the MoveIt group name (e.g. "manipulator")
        :type move_group: string
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
        self.move_group_namespace = move_group_namespace
        self.move_group = move_group
        self.eye_on_hand = eye_on_hand
        self.robot_base_frame = robot_base_frame
        self.robot_effector_frame = robot_effector_frame
        self.tracking_base_frame = tracking_base_frame
        self.tracking_marker_frame = tracking_marker_frame
        self.freehand_robot_movement = freehand_robot_movement

    @staticmethod
    def init_from_parameter_server(namespace):
        import rospy
        rospy.loginfo("Loading parameters for calibration {} from the parameters server".format(namespace))

        if not namespace.endswith('/'):
            namespace = namespace + '/'

        ret = HandeyeCalibrationParameters(namespace=namespace,
                                           move_group_namespace=rospy.get_param(namespace + 'move_group_namespace'),
                                           move_group=rospy.get_param(namespace + 'move_group'),
                                           eye_on_hand=rospy.get_param(namespace + 'eye_on_hand'),
                                           robot_effector_frame=rospy.get_param(namespace + 'robot_effector_frame'),
                                           robot_base_frame=rospy.get_param(namespace + 'robot_base_frame'),
                                           tracking_base_frame=rospy.get_param(namespace + 'tracking_base_frame'),
                                           tracking_marker_frame=rospy.get_param(namespace + 'tracking_marker_frame'),
                                           freehand_robot_movement=rospy.get_param(namespace + 'freehand_robot_movement'))
        return ret

    @staticmethod
    def store_to_parameter_server(parameters):
        import rospy
        namespace = parameters.namespace
        rospy.loginfo("Storing parameters for calibration {} into the parameters server".format(namespace))

        rospy.set_param(namespace + 'move_group_namespace', parameters.move_group_namespace)
        rospy.set_param(namespace + 'move_group', parameters.move_group)
        rospy.set_param(namespace + 'eye_on_hand', parameters.eye_on_hand)
        rospy.set_param(namespace + 'robot_effector_frame', parameters.robot_effector_frame)
        rospy.set_param(namespace + 'robot_base_frame', parameters.robot_base_frame)
        rospy.set_param(namespace + 'tracking_base_frame', parameters.tracking_base_frame)
        rospy.set_param(namespace + 'tracking_marker_frame', parameters.tracking_marker_frame)

    @staticmethod
    def from_dict(in_dict):
        return HandeyeCalibrationParameters(**in_dict)

    @staticmethod
    def to_dict(parameters):
        return vars(parameters)


class HandeyeCalibration(object):
    """
    Stores parameters and transformation of a hand-eye calibration for publishing.
    """
    DIRECTORY = os.path.expanduser('~/.ros/easy_handeye')
    """Default directory for calibration yaml files."""

    # TODO: use the HandeyeCalibration message instead, this should be HandeyeCalibrationConversions
    def __init__(self,
                 calibration_parameters=None,
                 transformation=None):
        """
        Creates a HandeyeCalibration object.

        :param transformation: transformation between optical origin and base/tool robot frame as tf tuple
        :type transformation: ((float, float, float), (float, float, float, float))
        :return: a HandeyeCalibration object

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        self.parameters = calibration_parameters

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
            self.transformation.header.frame_id = calibration_parameters.robot_effector_frame
        else:
            self.transformation.header.frame_id = calibration_parameters.robot_base_frame
        self.transformation.child_frame_id = calibration_parameters.tracking_base_frame

    @staticmethod
    def to_dict(calibration):
        """
        Returns a dictionary representing this calibration.

        :return: a dictionary representing this calibration.

        :rtype: dict[string, string|dict[string,float]]
        """
        ret = {
            'parameters': HandeyeCalibrationParameters.to_dict(calibration.parameters),
            'transformation': {
                'x': calibration.transformation.transform.translation.x,
                'y': calibration.transformation.transform.translation.y,
                'z': calibration.transformation.transform.translation.z,
                'qx': calibration.transformation.transform.rotation.x,
                'qy': calibration.transformation.transform.rotation.y,
                'qz': calibration.transformation.transform.rotation.z,
                'qw': calibration.transformation.transform.rotation.w
            }
        }

        return ret

    @staticmethod
    def from_dict(in_dict):
        """
        Sets values parsed from a given dictionary.

        :param in_dict: input dictionary.
        :type in_dict: dict[string, string|dict[string,float]]

        :rtype: None
        """
        tr = in_dict['transformation']
        ret = HandeyeCalibration(calibration_parameters=HandeyeCalibrationParameters.from_dict(in_dict['parameters']),
                                 transformation=((tr['x'], tr['y'], tr['z']), (tr['qx'], tr['qy'], tr['qz'], tr['qw'])))
        return ret

    @staticmethod
    def to_yaml(calibration):
        """
        Returns a yaml string representing this calibration.

        :return: a yaml string

        :rtype: string
        """
        return yaml.dump(HandeyeCalibration.to_dict(calibration), default_flow_style=False)

    @staticmethod
    def from_yaml(in_yaml):
        """
        Parses a yaml string and sets the contained values in this calibration.

        :param in_yaml: a yaml string
        :rtype: None
        """
        return HandeyeCalibration.from_dict(yaml.load(in_yaml))

    @staticmethod
    def init_from_parameter_server(namespace):
        import rospy
        rospy.loginfo("Loading calibration {} from the parameters server".format(namespace))

        params = HandeyeCalibrationParameters.init_from_parameter_server(namespace)

        ret = HandeyeCalibration(calibration_parameters=params,
                                 transformation=((
                                                     rospy.get_param(namespace + 'transformation/x'),
                                                     rospy.get_param(namespace + 'transformation/y'),
                                                     rospy.get_param(namespace + 'transformation/z'),
                                                 ), (

                                                     rospy.get_param(namespace + 'transformation/qx'),
                                                     rospy.get_param(namespace + 'transformation/qy'),
                                                     rospy.get_param(namespace + 'transformation/qz'),
                                                     rospy.get_param(namespace + 'transformation/qw'),
                                                 )))
        return ret

    @staticmethod
    def store_to_parameter_server(calibration):
        import rospy
        namespace = calibration.parameters.namespace
        t = calibration.transformation.transform
        
        rospy.loginfo("Storing calibration {} into the parameters server".format(namespace))

        HandeyeCalibrationParameters.store_to_parameter_server(calibration.parameters)

        rospy.set_param(namespace + 'transformation/x', t.translation.x)
        rospy.set_param(namespace + 'transformation/y', t.translation.y)
        rospy.set_param(namespace + 'transformation/z', t.translation.z)

        rospy.set_param(namespace + 'transformation/x', t.rotation.x)
        rospy.set_param(namespace + 'transformation/y', t.rotation.y)
        rospy.set_param(namespace + 'transformation/z', t.rotation.z)
        rospy.set_param(namespace + 'transformation/w', t.rotation.w)

    def filename(self):
        return HandeyeCalibration.filename_for_namespace(self.parameters.namespace)

    @staticmethod
    def filename_for_namespace(namespace):
        return HandeyeCalibration.DIRECTORY + '/' + namespace.rstrip('/').split('/')[-1] + '.yaml'

    @staticmethod
    def to_file(calibration):
        """
        Saves this calibration in a yaml file in the default path.

        The default path consists of the default directory and the namespace the node is running in.

        :rtype: None
        """
        if not os.path.exists(HandeyeCalibration.DIRECTORY):
            os.makedirs(HandeyeCalibration.DIRECTORY)

        with open(calibration.filename(), 'w') as calib_file:
            calib_file.write(HandeyeCalibration.to_yaml(calibration))

    @staticmethod
    def from_file(namespace):
        """
        Parses a yaml file in the default path and sets the contained values in this calibration.

        The default path consists of the default directory and the namespace the node is running in.

        :rtype: None
        """

        with open(HandeyeCalibration.filename_for_namespace(namespace)) as calib_file:
            return HandeyeCalibration.from_yaml(calib_file.read())
