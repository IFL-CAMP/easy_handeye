import rospy
import tf2_ros
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
        self.tfBuffer = tf2_ros.Buffer()
        """
        used to get transforms to build each sample

        :type: tf2_ros.Buffer
        """
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        """
        used to get transforms to build each sample

        :type: tf2_ros.TransformListener
        """
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        """
        used to publish the calibration after saving it

        :type: tf.TransformBroadcaster
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

    def _wait_for_tf_init(self):
        """
        Waits until all needed frames are present in tf.

        :rtype: None
        """
        self.tfBuffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame, rospy.Time(0),
                                       rospy.Duration(20))
        self.tfBuffer.lookup_transform(self.tracking_base_frame, self.tracking_marker_frame, rospy.Time(0),
                                       rospy.Duration(60))

    def _get_transforms(self, time=None):
        """
        Samples the transforms at the given time.

        :param time: sampling time (now if None)
        :type time: None|rospy.Time
        :rtype: dict[str, ((float, float, float), (float, float, float, float))]
        """
        if time is None:
            time = rospy.Time(0)

        # here we trick the library (it is actually made for eye_on_hand only). Trust me, I'm an engineer
        if self.eye_on_hand:
            rob = self.tfBuffer.lookup_transform(self.robot_base_frame, self.robot_effector_frame, time,
                                                 rospy.Duration(10))
        else:
            rob = self.tfBuffer.lookup_transform(self.robot_effector_frame, self.robot_base_frame, time,
                                                 rospy.Duration(10))
        opt = self.tfBuffer.lookup_transform(self.tracking_base_frame, self.tracking_marker_frame, time,
                                             rospy.Duration(10))
        return {'robot': rob, 'optical': opt}

    def take_sample(self):
        """
        Samples the transformations and appends the sample to the list.

        :rtype: None
        """
        rospy.loginfo("Taking a sample...")
        transforms = self._get_transforms()
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
            camera_marker_samples.transforms.append(s['optical'].transform)
            hand_world_samples.transforms.append(s['robot'].transform)

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
