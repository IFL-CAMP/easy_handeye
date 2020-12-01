import cv2
import numpy as np

import transforms3d as tfs
from rospy import logerr, logwarn, loginfo

from easy_handeye.handeye_calibration import HandeyeCalibration


class HandeyeCalibrationBackendOpenCV(object):
    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""

    AVAILABLE_ALGORITHMS = {
        'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
        'Park': cv2.CALIB_HAND_EYE_PARK,
        'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
        'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    @staticmethod
    def _msg_to_opencv(transform_msg):
        cmt = transform_msg.translation
        tr = np.array((cmt.x, cmt.y, cmt.z))
        cmq = transform_msg.rotation
        rot = tfs.quaternions.quat2mat((cmq.w, cmq.x, cmq.y, cmq.z))
        return rot, tr

    @staticmethod
    def _get_opencv_samples(samples):
        """
        Returns the sample list as a rotation matrix and a translation vector.

        :rtype: (np.array, np.array)
        """
        hand_base_rot = []
        hand_base_tr = []
        marker_camera_rot = []
        marker_camera_tr = []

        for s in samples:
            camera_marker_msg = s['optical'].transform
            (mcr, mct) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(camera_marker_msg)
            marker_camera_rot.append(mcr)
            marker_camera_tr.append(mct)

            base_hand_msg = s['robot'].transform
            (hbr, hbt) = HandeyeCalibrationBackendOpenCV._msg_to_opencv(base_hand_msg)
            hand_base_rot.append(hbr)
            hand_base_tr.append(hbt)

        return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

    def compute_calibration(self, handeye_parameters, samples, algorithm=None):
        """
        Computes the calibration through the OpenCV library and returns it.

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if algorithm is None:
            algorithm = 'Tsai-Lenz'

        loginfo('OpenCV backend calibrating with algorithm {}'.format(algorithm))

        if len(samples) < HandeyeCalibrationBackendOpenCV.MIN_SAMPLES:
            logwarn("{} more samples needed! Not computing the calibration".format(
                HandeyeCalibrationBackendOpenCV.MIN_SAMPLES - len(samples)))
            return

        # Update data
        opencv_samples = HandeyeCalibrationBackendOpenCV._get_opencv_samples(samples)
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = opencv_samples

        if len(hand_world_rot) != len(marker_camera_rot):
            logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        loginfo("Computing from %g poses..." % len(samples))

        method = HandeyeCalibrationBackendOpenCV.AVAILABLE_ALGORITHMS[algorithm]

        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(hand_world_rot, hand_world_tr, marker_camera_rot,
                                                               marker_camera_tr, method=method)
        result = tfs.affines.compose(np.squeeze(hand_camera_tr), hand_camera_rot, [1, 1, 1])

        loginfo("Computed calibration: {}".format(str(result)))
        (hcqw, hcqx, hcqy, hcqz) = [float(i) for i in tfs.quaternions.mat2quat(hand_camera_rot)]
        (hctx, hcty, hctz) = [float(i) for i in hand_camera_tr]

        result_tuple = ((hctx, hcty, hctz), (hcqx, hcqy, hcqz, hcqw))

        ret = HandeyeCalibration(calibration_parameters=handeye_parameters,
                                 transformation=result_tuple)

        return ret
