#!/usr/bin/env python

import rospy

from handeyecalibration.handeye_server import HandeyeServer


def main():
    rospy.init_node('handeyecalibration')
    while rospy.get_time() == 0.0:
        pass

    cw = HandeyeServer()

    rospy.spin()

if __name__ == '__main__':
    main()
