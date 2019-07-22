import rospy
import tf2_py
import tf2_ros
from geometry_msgs.msg import Vector3, Quaternion, Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
from easy_handeye.handeye_calibration import HandeyeCalibration


class HandeyeCalibrator(object):
    """
    Connects tf and ViSP hand2eye to provide an interactive mean of calibration.
    """

    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""

    def __init__(self):
        self.eye_on_hand = rospy.get_param('eye_on_hand', False)
        """
        if false, it is a eye-on-base calibration

        :type: bool
        """

        # tf names
        self.robot_effector_frame = rospy.get_param('robot_effector_frame', 'tool0')
        """
        robot tool tf name

        :type: string
        """
        self.robot_base_frame = rospy.get_param('robot_base_frame', 'base_link')
        """
        robot base tf name

        :type: str
        """
        self.tracking_base_frame = rospy.get_param('tracking_base_frame', 'optical_origin')
        """
        tracking system tf name

        :type: str
        """
        self.tracking_marker_frame = rospy.get_param('tracking_marker_frame', 'optical_target')
        """
        tracked object tf name

        :type: str
        """

        # tf structures
        self.buffer = tf2_ros.Buffer()
        """
        used to propagate changes to when tf info updates
        
        :type: tf2_ros.Buffer
        """

        self.listener = tf2_ros.TransformListener(self.buffer)
        """
        used to get transforms to build each sample

        :type: tf2_ros.TransformListener
        """
        self.broadcaster = tf2_ros.TransformBroadcaster()
        """
        used to publish the calibration after saving it

        :type: tf2_ros.TransformBroadcaster
        """

        # internal input data
        self.samples = []
        """
        list of acquired samples

        Each sample is a dictionary going from 'rob' and 'opt' to the relative sampled transform in tf tuple format.

        :type: list[dict[str, ((float, float, float), (float, float, float, float))]]
        """

        # calibration service
        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy(
            'compute_effector_camera_quick',
            compute_effector_camera_quick)
        """
        proxy to a ViSP hand2eye calibration service

        :type: rospy.ServiceProxy
        """

    def _wait_for_transforms(self, rate=rospy.Duration(1)):
        """
        Waits until the needed transformations are available in tf.

        :param rate: sampling rate
        :type rate: rospy.Duration
        :rtype: dict[str, ((float, float, float), (float, float, float, float))]
        """
        while not rospy.is_shutdown():
            try:
                return self._get_transforms(rate)

            except (tf2_py.LookupException, tf2_py.ConnectivityException, tf2_py.ExtrapolationException):
                rospy.sleep(rate)
                continue

    def _get_transforms(self, rate):
        """
        Tries to sample the transforms at the given rate.

        :param rate: sampling rate
        :type rate: rospy.Duration
        :rtype: dict[str, ((float, float, float), (float, float, float, float))]
        """
        # here we trick the library (it is actually made for eye_on_hand only). Trust me, I'm an engineer
        if self.eye_on_hand:
            rob = self.buffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame,
                                               rospy.Time.now(), rate).transform
        else:
            rob = self.buffer.lookup_transform(self.robot_effector_frame, self.robot_base_frame,
                                               rospy.Time.now(), rate).transform
        opt = self.buffer.lookup_transform(self.tracking_base_frame, self.tracking_marker_frame,
                                           rospy.Time.now(), rate).transform

        return {'robot': rob, 'optical': opt}

    def take_sample(self):
        """
        Samples the transformations and appends the sample to the list.

        :rtype: None
        """
        rospy.loginfo("Taking a sample...")
        transforms = self._wait_for_transforms()
        rospy.loginfo("Got a sample")
        self.samples.append(transforms)

    def remove_sample(self, index):
        """
        Removes a sample from the list.

        :type index: int
        :rtype: None
        """
        if 0 <= index < len(self.samples):
            del self.samples[index]

    def get_visp_samples(self):
        """
        Returns the sample list as a TransformArray.

        :rtype: visp_hand2eye_calibration.msg.TransformArray
        """
        hand_world_samples = TransformArray()
        # hand_world_samples.header.frame_id = self.robot_base_frame
        hand_world_samples.header.frame_id = self.tracking_base_frame
        # DONTFIXME: yes, I know, it should be like the line above.
        # thing is, otherwise the results of the calibration are wrong. don't ask me why

        camera_marker_samples = TransformArray()
        camera_marker_samples.header.frame_id = self.tracking_base_frame

        for s in self.samples:
            to = tf2_ros.convert(s['optical'], Transform)
            camera_marker_samples.transforms.append(to)
            tr = tf2_ros.convert(s['robot'], Transform)
            hand_world_samples.transforms.append(tr)

        return hand_world_samples, camera_marker_samples

    def compute_calibration(self):
        """
        Computes the calibration through the ViSP service and returns it.

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if len(self.samples) < HandeyeCalibrator.MIN_SAMPLES:
            rospy.logwarn("{} more samples needed! Not computing the calibration".format(
                HandeyeCalibrator.MIN_SAMPLES - len(self.samples)))
            return

        # Update data
        hand_world_samples, camera_marker_samples = self.get_visp_samples()

        if len(hand_world_samples.transforms) != len(camera_marker_samples.transforms):
            rospy.logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        rospy.loginfo("Computing from %g poses..." % len(self.samples))

        try:
            result = self.calibrate(camera_marker_samples, hand_world_samples)
            rospy.loginfo("Computed calibration: {}".format(str(result)))
            transl = result.effector_camera.translation
            rot = result.effector_camera.rotation
            result_tuple = ((transl.x, transl.y, transl.z),
                            (rot.x, rot.y, rot.z, rot.w))

            ret = HandeyeCalibration(self.eye_on_hand,
                                     self.robot_base_frame,
                                     self.robot_effector_frame,
                                     self.tracking_base_frame,
                                     result_tuple)

            return ret

        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: " + str(ex))
            return None
