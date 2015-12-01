
import os
import yaml
import rospy
import tf
import tf_conversions
from tf import transformations as tfs
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped


class HandeyePersistence(object):

    # transformation is tuple ((tx,ty,tz),(rx,ry,rz,rw)) as returned by lookupTransform
    def __init__(self,
                 eye_on_hand=False,
                 base_link_frame=None,
                 tool_frame=None,
                 optical_origin_frame=None,
                 transformation=None):

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

        a = tf.TransformerROS()
        tf_conversions.fromTf()

        ret = {
            'eye_on_hand': self.eye_on_hand,
            'optical_origin_frame': self.optical_origin_frame,
            'transformation': {
                'x': self.transformation[0][0],
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
        else:
            ret['base_link_frame'] = self.base_link_frame

        return ret

    def from_dict(self, in_dict):
        self.eye_on_hand = in_dict['']

    def from_parameters(self, namespace, eye_on_hand):
        prefix = namespace

        if self.eye_on_hand:
            rospy.get_param('tool_frame', self.tool_frame)
            rospy.get_param('optical_origin_frame', self.optical_origin_frame)
            prefix = 'tool_to_camera'
        else:
            rospy.get_param('optical_origin_frame', self.optical_origin_frame)
            rospy.get_param('base_link_frame', self.base_link_frame)
            prefix = 'base_to_camera'

        rospy.get_param(prefix + '_x')
        rospy.get_param(prefix + '_y')
        rospy.get_param(prefix + '_z')
        rospy.get_param(prefix + '_qx')
        rospy.get_param(prefix + '_qy')
        rospy.get_param(prefix + '_qz')
        rospy.get_param(prefix + '_qw')

    def to_parameters(self, namespace, calibration):
        prefix = namespace

        rospy.set_param('optical_origin_frame', self.optical_origin_frame)

        if self.eye_on_hand:
            rospy.set_param('tool_frame', self.tool_frame)
            prefix = 'tool_to_camera'
        else:
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

        with open(filename, 'w') as calib_file:
            calib_file.write(yaml.dump(calibration))

    def from_file(self, namespace):
        raise NotImplementedError