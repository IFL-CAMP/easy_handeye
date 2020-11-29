from __future__ import print_function
from __future__ import division
import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from easy_handeye.handeye_client import HandeyeClient

try:
    from python_qt_binding.QtGui import QWidget, QListWidgetItem
except ImportError:
    try:
        from python_qt_binding.QtWidgets import QWidget, QListWidgetItem, QLabel, QComboBox, QHBoxLayout
    except:
        raise ImportError('Could not import QWidgets')


def format_sample(sample):
    x, y, z = sample.translation.x, sample.translation.y, sample.translation.z
    qx, qy, qz, qw = sample.rotation.x, sample.rotation.y, sample.rotation.z, sample.rotation.w
    return 'translation: [{:+.2f}, {:+.2f}, {:+.2f}]\nrotation: [{:+.2f}, {:+.2f}, {:+.2f}, {:+.2f}]'.format(x, y, z, qx, qy, qz, qw)


class RqtHandeyeCalibration(Plugin):
    def __init__(self, context):
        super(RqtHandeyeCalibration, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('HandeyeCalibration')

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

        # Create QWidgets
        self._widget = QWidget()
        self._infoWidget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_easy_handeye'), 'resource', 'rqt_handeye.ui')
        ui_info_file = os.path.join(rospkg.RosPack().get_path('rqt_easy_handeye'), 'resource', 'rqt_handeye_info.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        loadUi(ui_info_file, self._infoWidget)
        self._widget.horizontalLayout_infoAndActions.insertWidget(0, self._infoWidget)

        self._calibration_algorithm_combobox = QComboBox()
        self._calibration_algorithm_layout = QHBoxLayout()
        self._calibration_algorithm_layout.insertWidget(0, QLabel('Calibration algorithm: '))
        self._calibration_algorithm_layout.insertWidget(1, self._calibration_algorithm_combobox)
        self._infoWidget.layout().insertLayout(-1, self._calibration_algorithm_layout)

        # Give QObjects reasonable names
        self._widget.setObjectName('RqtHandeyeCalibrationUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.client = HandeyeClient()

        resp = self.client.list_algorithms()
        for i, a in enumerate(resp.algorithms):
            self._calibration_algorithm_combobox.insertItem(i, a)
        index_of_curr_alg = resp.algorithms.index(resp.current_algorithm)
        self._calibration_algorithm_combobox.setCurrentIndex(index_of_curr_alg)
        self._calibration_algorithm_combobox.currentTextChanged.connect(self.client.set_algorithm)

        self._infoWidget.calibNameLineEdit.setText(rospy.get_namespace())
        self._infoWidget.trackingBaseFrameLineEdit.setText(self.client.parameters.tracking_base_frame)
        self._infoWidget.trackingMarkerFrameLineEdit.setText(self.client.parameters.tracking_marker_frame)
        self._infoWidget.robotBaseFrameLineEdit.setText(self.client.parameters.robot_base_frame)
        self._infoWidget.robotEffectorFrameLineEdit.setText(self.client.parameters.robot_effector_frame)
        if self.client.parameters.eye_on_hand:
            self._infoWidget.calibTypeLineEdit.setText("eye on hand")
        else:
            self._infoWidget.calibTypeLineEdit.setText("eye on base")

        self._widget.takeButton.clicked[bool].connect(self.handle_take_sample)
        self._widget.removeButton.clicked[bool].connect(self.handle_remove_sample)
        self._widget.computeButton.clicked[bool].connect(self.handle_compute_calibration)
        self._widget.saveButton.clicked[bool].connect(self.handle_save_calibration)

        self._widget.removeButton.setEnabled(False)
        self._widget.computeButton.setEnabled(False)
        self._widget.saveButton.setEnabled(False)

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

    def _display_sample_list(self, sample_list):
        self._widget.sampleListWidget.clear()

        for i in range(len(sample_list.hand_world_samples)):
            formatted_robot_sample = format_sample(sample_list.hand_world_samples[i])
            formatted_tracking_sample = format_sample(sample_list.camera_marker_samples[i])
            self._widget.sampleListWidget.addItem(
                '{}) \n hand->world \n {} \n camera->marker\n {}\n'.format(i + 1, formatted_robot_sample,
                                                                      formatted_tracking_sample))
        self._widget.sampleListWidget.setCurrentRow(len(sample_list.hand_world_samples) - 1)
        self._widget.removeButton.setEnabled(len(sample_list.hand_world_samples) > 0)

    def handle_take_sample(self):
        sample_list = self.client.take_sample()
        self._display_sample_list(sample_list)
        self._widget.computeButton.setEnabled(len(sample_list.hand_world_samples) > 1)
        self._widget.saveButton.setEnabled(False)

    def handle_remove_sample(self):
        index = self._widget.sampleListWidget.currentRow()
        sample_list = self.client.remove_sample(index)
        self._display_sample_list(sample_list)
        self._widget.computeButton.setEnabled(len(sample_list.hand_world_samples) > 1)
        self._widget.saveButton.setEnabled(False)

    def handle_compute_calibration(self):
        result = self.client.compute_calibration()
        self._widget.computeButton.setEnabled(False)
        if result.valid:
            self._widget.outputBox.setPlainText(str(result.calibration.transform.transform))
            self._widget.saveButton.setEnabled(True)
        else:
            self._widget.outputBox.setPlainText('The calibration could not be computed')
            self._widget.saveButton.setEnabled(False)

    def handle_save_calibration(self):
        self.client.save()
        self._widget.saveButton.setEnabled(False)
