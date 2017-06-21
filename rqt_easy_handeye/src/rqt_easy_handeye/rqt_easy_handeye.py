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
        from python_qt_binding.QtWidgets import QWidget, QListWidgetItem
    except:
        raise ImportError('Could not import QWidgets')


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
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_easy_handeye'), 'resource', 'rqt_handeye.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
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

        self._widget.calibNameLineEdit.setText(rospy.get_namespace())
        self._widget.trackingBaseFrameLineEdit.setText(self.client.tracking_base_frame)
        self._widget.trackingMarkerFrameLineEdit.setText(self.client.tracking_marker_frame)
        self._widget.robotBaseFrameLineEdit.setText(self.client.robot_base_frame)
        self._widget.robotEffectorFrameLineEdit.setText(self.client.robot_effector_frame)
        if self.client.eye_on_hand:
            self._widget.calibTypeLineEdit.setText("eye on hand")

        else:
            self._widget.calibTypeLineEdit.setText("eye on base")


        self._widget.takeButton.clicked[bool].connect(self.handle_take_sample)
        self._widget.removeButton.clicked[bool].connect(self.handle_remove_sample)
        self._widget.computeButton.clicked[bool].connect(self.handle_compute_calibration)
        self._widget.saveButton.clicked[bool].connect(self.handle_save_calibration)

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

        for i in range(len(sample_list.hand_world_samples.transforms)):
            self._widget.sampleListWidget.addItem(
                '{}) \n hand->world {} \n camera->marker {}\n'.format(i+1,
                                                                      sample_list.hand_world_samples.transforms[
                                                                          i],
                                                                      sample_list.camera_marker_samples.transforms[
                                                                          i]))
        self._widget.sampleListWidget.setCurrentRow(len(sample_list.hand_world_samples.transforms)-1)

    def handle_take_sample(self):
        sample_list = self.client.take_sample()
        self._display_sample_list(sample_list)

    def handle_remove_sample(self):
        index = self._widget.sampleListWidget.currentRow()
        sample_list = self.client.remove_sample(index)
        self._display_sample_list(sample_list)

    def handle_compute_calibration(self):
        result = self.client.compute_calibration()
        self._widget.outputBox.setPlainText(str(result.calibration.transform.transform))

    def handle_save_calibration(self):
        self.client.save()
