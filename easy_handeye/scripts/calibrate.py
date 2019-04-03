#!/usr/bin/env python

import rospy

from easy_handeye.handeye_server import HandeyeServer


def main():
    rospy.init_node('easy_handeye')
    while rospy.get_time() == 0.0:
        pass

    cw = HandeyeServer()

    rospy.spin()


if __name__ == '__main__':
    main()
