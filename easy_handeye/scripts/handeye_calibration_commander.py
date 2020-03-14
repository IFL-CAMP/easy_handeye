#!/usr/bin/env python

import rospy
import std_srvs
from std_srvs import srv
from easy_handeye.handeye_client import HandeyeClient


# for reading single character without hitting RETURN (unless it's ipython!)
def getchar():
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class HandeyeCalibrationCommander:
    def __init__(self):
        self.client = HandeyeClient()

    def _take_menu(self):
        print('Press SPACE to take a sample or ENTER to continue\n')
        i = getchar()
        if i == ' ':
            self.client.take_sample()

    def _display_sample_list(self, sample_list):
        for i in range(len(sample_list.hand_world_samples)):
            print('{}) \n hand->world {} \n camera->marker {}\n'.format(i,
                                       sample_list.hand_world_samples[i],
                                       sample_list.camera_marker_samples[i]))

    def _edit_menu(self):
        while len(self.client.get_sample_list().hand_world_samples) > 0:
            prompt_str = 'Press a number and ENTER to delete the respective sample, or ENTER to continue:\n'
            self._display_sample_list(self.client.get_sample_list())
            sample_to_delete = raw_input(prompt_str)
            if sample_to_delete.isdigit():
                self.client.remove_sample(int(sample_to_delete))
            else:
                break

    def _save_menu(self):
        print('Press c to compute the calibration or ENTER to continue\n')
        i = getchar()
        if i == 'c':
            cal = self.client.compute_calibration()
            print(cal)
        print('Press s to save the calibration to parameters and namespace, q to quit or ENTER to continue\n')
        i = getchar()
        if i == 's':
            self.client.save()
        elif i == 'q':
            quit()

    def _interactive_menu(self):
        self._take_menu()
        self._edit_menu()
        self._save_menu()

    def spin_interactive(self):
        self._edit_menu()  # the sample list might not be empty when we start the commander
        while not rospy.is_shutdown():
            self._interactive_menu()


def main():
    rospy.init_node('easy_handeye')
    while rospy.get_time() == 0.0:
        pass

    print('Handeye Calibration Commander')
    print('connecting to: {}'.format((rospy.get_namespace().strip('/'))))

    cmder = HandeyeCalibrationCommander()

    if cmder.client.eye_on_hand:
        print('eye-on-hand calibration')
        print('robot effector frame: {}'.format(cmder.client.robot_effector_frame))
    else:
        print('eye-on-base calibration')
        print('robot base frame: {}'.format(cmder.client.robot_base_frame))
    print('tracking base frame: {}'.format(cmder.client.tracking_base_frame))
    print('tracking target frame: {}'.format(cmder.client.tracking_marker_frame))

    cmder.spin_interactive()


if __name__ == '__main__':
    main()
