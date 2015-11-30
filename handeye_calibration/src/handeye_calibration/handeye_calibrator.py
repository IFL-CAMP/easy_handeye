import rospy
import std_msgs
import tf
from tf import transformations as tfs

from geometry_msgs.msg import Vector3, Quaternion, Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick


class HandeyeCalibrator(object):
    MIN_SAMPLES = 2  # TODO: correct?

    def __init__(self):

        self.eye_on_hand = rospy.get_param('eye_on_hand', False)

        # tf names
        self.base_link_frame = rospy.get_param('base_link_frame', 'base_link')
        self.tool_frame = rospy.get_param('tool_frame', 'tool0')
        self.optical_origin_frame = rospy.get_param('optical_origin_frame', 'optical_origin')
        self.optical_target_frame = rospy.get_param('optical_target_frame', 'optical_target')

        # tf structures
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.transformer = tf.TransformerROS()  # for converting messages to rotation matrices, etc.

        # inner input data
        self.samples = []

        # VISP input data
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()
        
        # calibration service
        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy(
            'compute_effector_camera_quick',
            compute_effector_camera_quick)

    @staticmethod
    def _tuple_to_visp_transform(tf_t):
        transl = Vector3(*tf_t[0])
        rot = Quaternion(*tf_t[1])
        return Transform(transl, rot)

    # TODO: find a reasonable name
    def _inner_to_visp_samples(self):
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()
        self.hand_world_samples.header.frame_id = self.optical_origin_frame 
        self.camera_marker_samples.header.frame_id = self.optical_origin_frame
        for s in self.samples:
            to = HandeyeCalibrator._tuple_to_visp_transform(s['optical'])
            self.camera_marker_samples.transforms.append(to)
            tr = HandeyeCalibrator._tuple_to_visp_transform(s['robot'])
            self.hand_world_samples.transforms.append(tr)

    def _wait_for_tf_init(self):
        self.listener.waitForTransform(self.base_link_frame, self.tool_frame, rospy.Time(0), rospy.Duration(10))
        self.listener.waitForTransform(self.optical_origin_frame, self.optical_target_frame, rospy.Time(0),
                                       rospy.Duration(60))

    def _wait_for_transforms(self):
        now = rospy.Time.now()
        self.listener.waitForTransform(self.base_link_frame, self.tool_frame, now, rospy.Duration(10))
        self.listener.waitForTransform(self.optical_origin_frame, self.optical_target_frame, now, rospy.Duration(10))
        return now

    def _get_transforms(self, time=None):
        if time is None:
            time = self._wait_for_transforms()

        rob = None
        if self.eye_on_hand:
            rob = self.listener.lookupTransform(self.base_link_frame, self.tool_frame,
                                            time)
        else:
            rob = self.listener.lookupTransform(self.tool_frame, self.base_link_frame,
                                            time)
        opt = self.listener.lookupTransform(self.optical_origin_frame, self.optical_target_frame, time)
        return {'robot': rob, 'optical': opt}

    def take_sample(self):
        rospy.loginfo("Getting transforms")
        transforms = self._get_transforms()
        rospy.loginfo("Got transforms")
        self.samples.append(transforms)

    def remove_sample(self, index):
            del self.samples[index]

    def compute_calibration(self):

        if len(self.samples) < HandeyeCalibrator.MIN_SAMPLES:
            rospy.logwarn("%d more samples needed..." % (HandeyeCalibrator.MIN_SAMPLES - len(self.samples)))
            return

        # Update data
        self._inner_to_visp_samples()

        if len(self.hand_world_samples.transforms) != len(self.camera_marker_samples.transforms):
            rospy.logerr("Different numbers of hand-world and camera-marker samples.")
            return

        rospy.loginfo("Computing from %g poses..." % len(self.samples))
        result = None

        try:
            result = self.calibrate(self.camera_marker_samples, self.hand_world_samples)
        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: " + str(ex))
            return None

        transl = result.effector_camera.translation
        rot = result.effector_camera.rotation
        result_tf = Transform((transl.x,
                               transl.y,
                               transl.z),
                              (rot.x,
                               rot.y,
                               rot.z,
                               rot.w))

        cal_mat = self.transformer.fromTranslationRotation(result_tf.translation,
                                                           result_tf.rotation)
        cal_mat_inv = tfs.inverse_matrix(cal_mat)
        transl = tfs.translation_from_matrix(cal_mat_inv)
        rot = tfs.quaternion_from_matrix(cal_mat_inv)
        camera_to_base = Transform(Vector3(transl[0],
                                           transl[1],
                                           transl[2]),
                                   Quaternion(rot[0],
                                              rot[1],
                                              rot[2],
                                              rot[3]))

        ret = {
            'transformation': {
                'x': result.effector_camera.translation.x,
                'y': result.effector_camera.translation.y,
                'z': result.effector_camera.translation.z,
                'qx': result.effector_camera.rotation.x,
                'qy': result.effector_camera.rotation.y,
                'qz': result.effector_camera.rotation.z,
                'qw': result.effector_camera.rotation.w
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

    def set_parameters(self, calibration):
        prefix = None

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
