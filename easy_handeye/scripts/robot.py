#!/usr/bin/env python

import rospy

from easy_handeye.handeye_server_robot import HandeyeServerRobot


def main():
    rospy.init_node('easy_handeye_calibration_server_robot')
    while rospy.get_time() == 0.0:
        pass

    calibration_namespace=rospy.get_param('~calibration_namespace')

    cw = HandeyeServerRobot(namespace=calibration_namespace)

    rospy.spin()


if __name__ == '__main__':
    main()
