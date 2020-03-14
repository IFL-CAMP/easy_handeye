from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rospy import Time, Duration, loginfo


class HandeyeSampler(object):
    """
    Manages the samples acquired from tf.
    """

    def __init__(self, handeye_parameters):
        self.handeye_parameters = handeye_parameters

        # tf structures
        self.tfBuffer = Buffer()
        """
        used to get transforms to build each sample

        :type: tf2_ros.Buffer
        """
        self.tfListener = TransformListener(self.tfBuffer)
        """
        used to get transforms to build each sample

        :type: tf2_ros.TransformListener
        """
        self.tfBroadcaster = TransformBroadcaster()
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

    def _wait_for_tf_init(self):
        """
        Waits until all needed frames are present in tf.

        :rtype: None
        """
        self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                       self.handeye_parameters.robot_effector_frame, Time(0),
                                       Duration(20))
        self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                       self.handeye_parameters.tracking_marker_frame, Time(0),
                                       Duration(60))

    def _get_transforms(self, time=None):
        """
        Samples the transforms at the given time.

        :param time: sampling time (now if None)
        :type time: None|Time
        :rtype: dict[str, ((float, float, float), (float, float, float, float))]
        """
        if time is None:
            time = Time.now()

        # here we trick the library (it is actually made for eye_on_hand only). Trust me, I'm an engineer
        if self.handeye_parameters.eye_on_hand:
            rob = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                                 self.handeye_parameters.robot_effector_frame, time,
                                                 Duration(10))
        else:
            rob = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_effector_frame,
                                                 self.handeye_parameters.robot_base_frame, time,
                                                 Duration(10))
        opt = self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                             self.handeye_parameters.tracking_marker_frame, time,
                                             Duration(10))
        return {'robot': rob, 'optical': opt}

    def take_sample(self):
        """
        Samples the transformations and appends the sample to the list.

        :rtype: None
        """
        loginfo("Taking a sample...")
        transforms = self._get_transforms()
        loginfo("Got a sample")
        self.samples.append(transforms)

    def remove_sample(self, index):
        """
        Removes a sample from the list.

        :type index: int
        :rtype: None
        """
        if 0 <= index < len(self.samples):
            del self.samples[index]

    def get_samples(self):
        """
        Returns the samples accumulated so far.
        :rtype: [dict[str, ((float, float, float), (float, float, float, float))]]
        :return: A list of tuples containing the tracking and the robot transform pairs
        """
        return self.samples
