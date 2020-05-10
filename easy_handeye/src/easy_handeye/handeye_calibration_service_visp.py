from rospy import wait_for_service, logerr, logwarn, loginfo, ServiceProxy, ServiceException
from easy_handeye.handeye_calibration import HandeyeCalibration
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick


class HandeyeCalibrationServiceVisp(object):
    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""

    def __init__(self):
        # calibration service
        wait_for_service('compute_effector_camera_quick')
        self.calibrate = ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)
        """
        proxy to a ViSP hand2eye calibration service

        :type: rospy.ServiceProxy
        """

    @staticmethod
    def get_visp_samples(handeye_parameters, samples):
        """
        Returns the sample list as a TransformArray.

        :rtype: visp_hand2eye_calibration.msg.TransformArray
        """
        hand_world_samples = TransformArray()
        # hand_world_samples.header.frame_id = handeye_parameters.robot_base_frame
        hand_world_samples.header.frame_id = handeye_parameters.tracking_base_frame
        # DONTFIXME: yes, I know, it should be like the line above.
        # thing is, otherwise the results of the calibration are wrong. don't ask me why

        camera_marker_samples = TransformArray()
        camera_marker_samples.header.frame_id = handeye_parameters.tracking_base_frame

        for s in samples:
            camera_marker_samples.transforms.append(s['optical'].transform)
            hand_world_samples.transforms.append(s['robot'].transform)

        return hand_world_samples, camera_marker_samples

    def compute_calibration(self, handeye_parameters, samples):
        """
        Computes the calibration through the ViSP service and returns it.

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if len(samples) < HandeyeCalibrationServiceVisp.MIN_SAMPLES:
            logwarn("{} more samples needed! Not computing the calibration".format(
                HandeyeCalibrationServiceVisp.MIN_SAMPLES - len(samples)))
            return

        # Update data
        hand_world_samples, camera_marker_samples = HandeyeCalibrationServiceVisp.get_visp_samples(samples=samples,
                                                                                                   handeye_parameters=handeye_parameters)

        if len(hand_world_samples.transforms) != len(camera_marker_samples.transforms):
            logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        loginfo("Computing from %g poses..." % len(samples))

        try:
            result = self.calibrate(camera_marker_samples, hand_world_samples)
            loginfo("Computed calibration: {}".format(str(result)))
            transl = result.effector_camera.translation
            rot = result.effector_camera.rotation
            result_tuple = ((transl.x, transl.y, transl.z),
                            (rot.x, rot.y, rot.z, rot.w))

            ret = HandeyeCalibration(calibration_parameters=handeye_parameters,
                                     transformation=result_tuple)

            return ret

        except ServiceException as ex:
            logerr("Calibration failed: " + str(ex))
            return None
