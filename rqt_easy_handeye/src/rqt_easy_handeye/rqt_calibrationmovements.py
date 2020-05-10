from __future__ import print_function
from __future__ import division
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QCoreApplication, Qt
from easy_handeye.handeye_client import HandeyeClient
try:
    from python_qt_binding.QtGui import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, \
        QPushButton
except ImportError:
    from python_qt_binding.QtWidgets import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QProgressBar, QLabel, \
        QPushButton
import rospy
import sys


class CalibrationMovementsGUI(QWidget):
    NOT_INITED_YET = 0
    BAD_PLAN = 1
    GOOD_PLAN = 2
    MOVED_TO_POSE = 3
    BAD_STARTING_POSITION = 4
    GOOD_STARTING_POSITION = 5
    CHECKING_STARTING_POSITION = 6
    MOVEMENT_FAILED = 7

    def __init__(self):
        super(CalibrationMovementsGUI, self).__init__()
        self.handeye_client = HandeyeClient()
        self.current_target_pose = -1  # -1 is home
        self.target_poses = None
        self.plan_was_successful = None
        self.state = CalibrationMovementsGUI.NOT_INITED_YET

        self.layout = QVBoxLayout()
        self.labels_layout = QHBoxLayout()
        self.buttons_layout = QHBoxLayout()

        self.progress_bar = QProgressBar()
        self.pose_number_lbl = QLabel('0/0')
        self.bad_plan_lbl = QLabel('No plan yet')
        self.bad_plan_lbl.setAlignment(Qt.AlignCenter)
        self.guide_lbl = QLabel('Hello')
        self.guide_lbl.setWordWrap(True)

        self.check_start_pose_btn = QPushButton('Check starting pose')
        self.check_start_pose_btn.clicked.connect(self.handle_check_current_state)

        self.next_pose_btn = QPushButton('Next Pose')
        self.next_pose_btn.clicked.connect(self.handle_next_pose)

        self.plan_btn = QPushButton('Plan')
        self.plan_btn.clicked.connect(self.handle_plan)

        self.execute_btn = QPushButton('Execute')
        self.execute_btn.clicked.connect(self.handle_execute)

        self.labels_layout.addWidget(self.pose_number_lbl)
        self.labels_layout.addWidget(self.bad_plan_lbl)

        self.buttons_layout.addWidget(self.check_start_pose_btn)
        self.buttons_layout.addWidget(self.next_pose_btn)
        self.buttons_layout.addWidget(self.plan_btn)
        self.buttons_layout.addWidget(self.execute_btn)

        self.layout.addWidget(self.progress_bar)
        self.layout.addLayout(self.labels_layout)
        self.layout.addWidget(self.guide_lbl)
        self.layout.addLayout(self.buttons_layout)

        self.setLayout(self.layout)

        self.plan_btn.setEnabled(False)
        self.execute_btn.setEnabled(False)

        self.setWindowTitle('Local Mover')
        self.show()

    def update_ui(self):
        if self.target_poses:
            count_target_poses = len(self.target_poses)
        else:
            count_target_poses = 1
        self.progress_bar.setMaximum(count_target_poses)
        self.progress_bar.setValue(self.current_target_pose + 1)
        self.pose_number_lbl.setText('{}/{}'.format(self.current_target_pose + 1, count_target_poses))

        if self.state == CalibrationMovementsGUI.BAD_PLAN:
            self.bad_plan_lbl.setText('BAD plan!! Don\'t do it!!!!')
            self.bad_plan_lbl.setStyleSheet('QLabel { background-color : red}')
        elif self.state == CalibrationMovementsGUI.GOOD_PLAN:
            self.bad_plan_lbl.setText('Good plan')
            self.bad_plan_lbl.setStyleSheet('QLabel { background-color : green}')
        else:
            self.bad_plan_lbl.setText('No plan yet')
            self.bad_plan_lbl.setStyleSheet('')

        if self.state == CalibrationMovementsGUI.NOT_INITED_YET:
            self.guide_lbl.setText(
                'Bring the robot to a plausible position and check if it is a suitable starting pose')
        elif self.state == CalibrationMovementsGUI.CHECKING_STARTING_POSITION:
            self.guide_lbl.setText(
                'Checking if the robot can translate and rotate in all directions from the current pose')
        elif self.state == CalibrationMovementsGUI.BAD_STARTING_POSITION:
            self.guide_lbl.setText('Cannot calibrate from current position')
        elif self.state == CalibrationMovementsGUI.GOOD_STARTING_POSITION:
            self.guide_lbl.setText('Ready to start: click on next pose')
        elif self.state == CalibrationMovementsGUI.GOOD_PLAN:
            self.guide_lbl.setText('The plan seems good: press execute to move the robot')
        elif self.state == CalibrationMovementsGUI.BAD_PLAN:
            self.guide_lbl.setText('Planning failed: try again')
        elif self.state == CalibrationMovementsGUI.MOVED_TO_POSE:
            self.guide_lbl.setText('Pose reached: take a sample and go on to next pose')

        can_plan = self.state == CalibrationMovementsGUI.GOOD_STARTING_POSITION
        self.plan_btn.setEnabled(can_plan)
        can_move = self.state == CalibrationMovementsGUI.GOOD_PLAN
        self.execute_btn.setEnabled(can_move)
        QCoreApplication.processEvents()

    def handle_check_current_state(self):
        self.state = CalibrationMovementsGUI.CHECKING_STARTING_POSITION
        self.update_ui()
        res = self.handeye_client.check_starting_pose()
        if res.can_calibrate:
            self.state = CalibrationMovementsGUI.GOOD_STARTING_POSITION
        else:
            self.state = CalibrationMovementsGUI.BAD_STARTING_POSITION
        self.current_target_pose = res.target_poses.current_target_pose_index
        self.target_poses = res.target_poses.target_poses
        self.plan_was_successful = None

        self.update_ui()

    def handle_next_pose(self):
        res = self.handeye_client.select_target_pose(self.current_target_pose+1)
        self.current_target_pose = res.target_poses.current_target_pose_index
        self.target_poses = res.target_poses.target_poses
        self.plan_was_successful = None

        self.state = CalibrationMovementsGUI.GOOD_STARTING_POSITION
        self.update_ui()

    def handle_plan(self):
        self.guide_lbl.setText('Planning to the next position. Click on execute when a good one was found')
        res = self.handeye_client.plan_to_selected_target_pose()
        self.plan_was_successful = res.success
        if self.plan_was_successful:
            self.state = CalibrationMovementsGUI.GOOD_PLAN
        else:
            self.state = CalibrationMovementsGUI.BAD_PLAN
        self.update_ui()

    def handle_execute(self):
        if self.plan_was_successful:
            self.guide_lbl.setText('Going to the selected pose')
            res = self.handeye_client.execute_plan()
            if res.success:
                self.state = CalibrationMovementsGUI.MOVED_TO_POSE
            else:
                self.state = CalibrationMovementsGUI.MOVEMENT_FAILED
            self.update_ui()


class RqtCalibrationMovements(Plugin):
    def __init__(self, context):
        super(RqtCalibrationMovements, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('LocalMover')

        rospy.sleep(1.0)

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = CalibrationMovementsGUI()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog


if __name__ == '__main__':

    NODE_NAME = 'easy_handeye_mover'

    rospy.init_node(NODE_NAME)
    while rospy.get_time() == 0.0:
        pass

    qapp = QApplication(sys.argv)
    lmg = CalibrationMovementsGUI()
    lmg.show()
    sys.exit(qapp.exec_())

    # TODO: alternative workflow for automatic calibration:
    # generate poses around current pose, each rotating by angle_delta in each direction
    # generate a plan to each pose and back
    # if all movements are possible, perform them in a row (at low speed)
