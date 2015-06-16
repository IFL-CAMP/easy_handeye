import rospy
import tf
from tf import transformations as tfs

from geometry_msgs.msg import Vector3, Quaternion, Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick


class HandEyeConnector(object):
    MIN_SAMPLES = 2  # TODO: correct?

    def __init__(self):

        self.eye_on_hand = rospy.get_param('eye_on_hand', False)
        self.publish_tf = rospy.get_param('publish_tf', True)
        self.sample_rate = rospy.get_param('sample_rate', 10)
        self.automatic = rospy.get_param('automatic', False)

        # tf names
        self.base_link_frame = rospy.get_param('base_link_frame', 'base_link')
        self.tool_frame = rospy.get_param('tool_frame', 'tool0')
        self.optical_origin_frame = rospy.get_param('optical_origin_frame', 'optical_origin')
        self.optical_target_frame = rospy.get_param('optical_target_frame', 'optical_target')

        # tf structures
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.transformer = tf.TransformerROS()  # for converting messages to rotation matrices, etc.

        # rate limiter
        self.rate = rospy.Rate(self.sample_rate)

        # inner input data
        self.samples = []

        # VISP input data
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()

        # automatic mode
        self.first_stable_time = None
        self.first_stable_sample = None
        self.auto_samples = []

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
            to = HandEyeConnector._tuple_to_visp_transform(s['optical'])
            self.camera_marker_samples.transforms.append(to)
            tr = HandEyeConnector._tuple_to_visp_transform(s['robot'])
            self.hand_world_samples.transforms.append(tr)

    def compute_calibration(self):

        if len(self.samples) < HandEyeConnector.MIN_SAMPLES:
            rospy.logwarn("%d more samples needed..." % (HandEyeConnector.MIN_SAMPLES - len(self.samples)))
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

        if self.publish_tf:  # TODO: rename to set_parameter
            prefix = None
            
            if self.eye_on_hand:
                rospy.set_param('tool_frame', self.tool_frame)
                rospy.set_param('optical_origin_frame', self.optical_origin_frame)
                prefix = 'tool_to_camera'
            else:
                rospy.set_param('optical_origin_frame', self.optical_origin_frame)
                rospy.set_param('base_link_frame', self.base_link_frame)
                prefix = 'base_to_camera'
            
            rospy.set_param(prefix + '_x', float(result.effector_camera.translation.x))
            rospy.set_param(prefix + '_y', float(result.effector_camera.translation.y))
            rospy.set_param(prefix + '_z', float(result.effector_camera.translation.z))
            rospy.set_param(prefix + '_qx', float(result.effector_camera.rotation.x))
            rospy.set_param(prefix + '_qy', float(result.effector_camera.rotation.y))
            rospy.set_param(prefix + '_qz', float(result.effector_camera.rotation.z))
            rospy.set_param(prefix + '_qw', float(result.effector_camera.rotation.w))

        rospy.loginfo("Result:\n" + str(camera_to_base))

        return camera_to_base

    def _edit_menu(self):
        prompt_str = 'Press a number and ENTER to delete the respective sample, or ENTER to continue:\n'
        for i in range(len(self.samples)):
            prompt_str += str(i + 1) + ' ' + str(self.samples[i]) + '\n'
        i = raw_input(prompt_str)
        if i.isdigit():
            del self.samples[i]

    def _save_menu(self):
        i = raw_input('Press c+ENTER to compute the calibration or ENTER to continue\n')
        if i == 'c':
            cal = self.compute_calibration()
            print(cal)
        i = raw_input('Press q+ENTER to quit or ENTER to continue\n')
        if i == 'q':
            quit()

    def _interactive_menu(self):
        self._edit_menu()
        self._save_menu()

    # TODO: compute new calibration if enough samples, output result, ask for editing list of samples or save / output
    def process_sample(self, msg):

        rospy.loginfo("Processing sample")

        self.samples.append(msg)

        # if not self.compute_calibration():
        # return

        # interactive
        if self.automatic:
            self.rate.sleep()
        else:
            self._interactive_menu()

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

    def spin_interactive(self):
        rospy.loginfo('Base link frame: ' + self.base_link_frame)
        rospy.loginfo('End effector frame: ' + self.tool_frame)
        rospy.loginfo('Optical origin frame: ' + self.optical_origin_frame)
        rospy.loginfo('Optical target frame: ' + self.optical_target_frame)
        rospy.loginfo('Waiting for transforms between frames')
        self._wait_for_tf_init()
        while not rospy.is_shutdown():
            try:
                raw_input('Hit a button to get a sample\n')
                try:
                    transforms = self._get_transforms()
                except tf.Exception as ex:
                    rospy.logwarn(str(ex))
                    continue
                self.process_sample(transforms)

            except KeyboardInterrupt:
                break

                # def _reset_auto_samples(self):
                # self.auto_samples = []
                # self.first_stable_time = self._wait_for_transforms()
                # self.first_stable_sample = self._get_transforms(self.first_stable_time)
                #     self.auto_samples.append(self.first_stable_sample)
                #
                # @staticmethod
                # def _similar_samples(sample1, sample2):
                #     # TODO: implement
                #     raise NotImplementedError
                #
                # def _enqueue_sample_if_similar(self, sample):
                #     if HandEyeConnector._similar_samples(self.first_stable_sample, sample):
                #         self._reset_auto_samples()
                #     else:
                #         self.auto_samples.append(sample)
                #
                # def spin_automatic(self):
                #     # TODO: watch TF for moments where the transforms are stable (use self.rate)
                #     # when one is detected, call process_sample
                #     raise NotImplementedError
                #     self._reset_auto_samples()
                #
                #     while not rospy.is_shutdown():
                #         sample = self._get_transforms()
                #         self._enqueue_sample_if_similar(sample)

