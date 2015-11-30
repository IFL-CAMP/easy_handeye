
import os
import rospy
import std_msgs
import tf
from tf import transformations as tfs

from geometry_msgs.msg import Vector3, Quaternion, Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick


class HandeyePersistence(object):

    def __init__(self, eye_on_hand=False, base_link_frame=None, tool_frame=None, optical_origin_frame=None, transformation=None):

        self.eye_on_hand = eye_on_hand

        # tf names
        if self.eye_on_hand:
            self.tool_frame = tool_frame
        else:
            self.base_link_frame = base_link_frame
        self.optical_origin_frame = optical_origin_frame

        self.transformation = transformation

    def to_yaml(self):

        ret = {
            'transformation': {
                'x': self.transformation.translation.x,
                'y': self.transformation.translation.y,
                'z': self.transformation.translation.z,
                'qx': self.transformation.rotation.x,
                'qy': self.transformation.rotation.y,
                'qz': self.transformation.rotation.z,
                'qw': self.transformation.rotation.w
            }
        }
        if self.eye_on_hand:
            ret['tool_frame'] = self.tool_frame
            ret['optical_origin_frame'] = self.optical_origin_frame
            ret['prefix'] = 'tool_to_camera'
        else:
            ret['optical_origin_frame'] = self.optical_origin_frame
            ret['base_link_frame'] = self.base_link_frame
            ret['prefix'] = 'base_to_camera'

        return ret

    def from_yaml(self):
        raise NotImplementedError

    def to_parameters(self, namespace, calibration):
        prefix = namespace

        if self.eye_on_hand:
            rospy.set_param('tool_frame', self.tool_frame)
            rospy.set_param('optical_origin_frame', self.optical_origin_frame)
            prefix = 'tool_to_camera'
        else:
            rospy.set_param('optical_origin_frame', self.optical_origin_frame)
            rospy.set_param('base_link_frame', self.base_link_frame)
            prefix = 'base_to_camera'

        rospy.set_param(prefix + '_x', calibration['transformation']['x'])
        rospy.set_param(prefix + '_y', calibration['transformation']['y'])
        rospy.set_param(prefix + '_z', calibration['transformation']['z'])
        rospy.set_param(prefix + '_qx', calibration['transformation']['qx'])
        rospy.set_param(prefix + '_qy', calibration['transformation']['qy'])
        rospy.set_param(prefix + '_qz', calibration['transformation']['qz'])
        rospy.set_param(prefix + '_qw', calibration['transformation']['qw'])

    def to_file(self, namespace, calibration):
        # TODO: save as yaml file
        directory = '~/.ros/handeye_calibration'
        if not os.path.exists(directory):
            os.makedirs(directory)
        filename = rospy.get_namespace()+'.yaml'

        raise NotImplementedError

        with open(filename) as calib_file:
            calib_file.write(yaml.dump(calibration))

    def from_file(self, namespace):
        raise NotImplementedError