#!/usr/bin/env python

import rospy


# for reading single character without hitting RETURN
def getchar(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def _edit_menu(self):
    sample_to_delete=None
    while sample_to_delete != '':
        prompt_str = 'Press a number and ENTER to delete the respective sample, or ENTER to continue:\n'
        for i in range(len(self.samples)):
            prompt_str += str(i + 1) + ' ' + str(self.samples[i]) + '\n'
        sample_to_delete = raw_input(prompt_str)
        if sample_to_delete.isdigit():
            del self.samples[int(sample_to_delete)-1]


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


def spin_interactive(self):
    rospy.loginfo('Base link frame: ' + self.base_link_frame)
    rospy.loginfo('End effector frame: ' + self.tool_frame)
    rospy.loginfo('Optical origin frame: ' + self.optical_origin_frame)
    rospy.loginfo('Optical target frame: ' + self.optical_target_frame)
    rospy.loginfo('Waiting for transforms between frames')
    self._wait_for_tf_init()
    while not rospy.is_shutdown():
         self._save_menu()


def main():
    rospy.init_node('handeye_calibration')
    while rospy.get_time() == 0.0: pass

    #TODO: command line interface


if __name__ == '__main__':
    main()
