import os
import yaml
import rospy
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped


class HandeyeCalibration(object):
    DIRECTORY = os.path.expanduser('~/.ros/handeyecalibration')

    # transformation is tuple ((tx,ty,tz),(rx,ry,rz,rw)) as returned by lookupTransform
    def __init__(self,
                 eye_on_hand=False,
                 base_link_frame=None,
                 tool_frame=None,
                 optical_origin_frame=None,
                 transformation=None):

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
        return yaml.dump(self.to_dict())

    def from_yaml(self, in_yaml):
        self.from_dict(yaml.load(in_yaml))

    def to_file(self):
        if not os.path.exists(HandeyeCalibration.DIRECTORY):
            os.makedirs(HandeyeCalibration.DIRECTORY)
        filename = HandeyeCalibration.DIRECTORY + rospy.get_namespace() + '.yaml'

        with open(filename, 'w') as calib_file:
            calib_file.write(self.to_yaml())

    def from_file(self):
        filename = HandeyeCalibration.DIRECTORY + rospy.get_namespace() + '.yaml'

        with open(filename) as calib_file:
            self.from_yaml(calib_file.read())

    def from_parameters(self):
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

