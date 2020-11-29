
#include <QLabel>
#include <QPainter>
#include <QTimer>

#include "calibration_panel.h"
//#include </home/marco/ros/catkin_ws/build/easy_handeye_rviz_plugins/easy_handeye_rviz_plugins_autogen/include/ui_calibration_panel.h>  // TODO remove
#include <ui_calibration_panel.h>

namespace easy_handeye_rviz_plugins
{
CalibrationPanel::CalibrationPanel(QWidget* parent) : rviz::Panel(parent)
{
  m_ui = new Ui_CalibrationPanel();

  m_ui->setupUi(this);

  for (const auto& calibrationNamespace : HandeyeClient::listRunningCalibrations())
  {
    m_ui->comboBox_namespace->addItem(QString::fromStdString(calibrationNamespace));
  }

  activateCalibration(m_ui->comboBox_namespace->currentText().toStdString());

  easy_handeye_msgs::SampleList sl;
  if (m_handeyeClient.getSampleList(sl))
    setSampleList(sl);

  updateUI();

  connect(m_ui->comboBox_namespace, QOverload<const QString&>::of(&QComboBox::currentIndexChanged),
          [this](const QString& newNamespace) { activateCalibration(newNamespace.toStdString()); });

  connect(m_ui->takeButton, &QPushButton::clicked, this, &CalibrationPanel::onTakeSamplePressed);
  connect(m_ui->removeButton, &QPushButton::clicked, this, &CalibrationPanel::onRemoveSamplePressed);
  connect(m_ui->saveButton, &QPushButton::clicked, this, &CalibrationPanel::onSaveCalibrationPressed);

  connect(m_ui->checkStartPoseButton, &QPushButton::clicked, this, &CalibrationPanel::onCheckStartingPosePressed);
}

void CalibrationPanel::activateCalibration(const std::string& calibrationNamespace)
{
  m_handeyeClient.selectCalibration(calibrationNamespace);
  m_robotMovementsClient.selectCalibration(calibrationNamespace);

  m_ui->calibNameLineEdit->setText(QString::fromStdString(calibrationNamespace));
  m_ui->calibTypeLineEdit->setText(m_handeyeClient.isEyeOnHand() ? "eye on hand" : "eye on base");
  m_ui->robotBaseFrameLineEdit->setText(QString::fromStdString(m_handeyeClient.robotBaseFrame()));
  m_ui->robotEffectorFrameLineEdit->setText(QString::fromStdString(m_handeyeClient.robotEffectorFrame()));
  m_ui->trackingBaseFrameLineEdit->setText(QString::fromStdString(m_handeyeClient.trackingBaseFrame()));
  m_ui->trackingMarkerFrameLineEdit->setText(QString::fromStdString(m_handeyeClient.trackingMarkerFrame()));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void CalibrationPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  //  config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void CalibrationPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  //  QString topic;
  //  if (config.mapGetString("Topic", &topic)) {
  //  }
}

void CalibrationPanel::onTakeSamplePressed(bool)
{
  m_ui->takeButton->setEnabled(false);
  QCoreApplication::processEvents();
  easy_handeye_msgs::SampleList new_sample_list;
  if (m_handeyeClient.takeSample(new_sample_list))
  {
    setSampleList(new_sample_list);
  }
  else
  {
    ROS_DEBUG("Could not get a new sample");
  }
  computeCalibration();
  m_ui->takeButton->setEnabled(true);
}

void CalibrationPanel::onRemoveSamplePressed(bool)
{
  m_ui->removeButton->setEnabled(false);
  int sample_index = m_ui->sampleListWidget->currentRow();
  easy_handeye_msgs::SampleList new_sample_list;
  if (m_handeyeClient.removeSample(sample_index, new_sample_list))
  {
    setSampleList(new_sample_list, sample_index);
  }
  else
  {
    ROS_DEBUG_STREAM("Could not remove sample " << sample_index);
  }
  m_ui->removeButton->setEnabled(true);
}

void CalibrationPanel::onSaveCalibrationPressed(bool)
{
  m_ui->saveButton->setEnabled(false);
  if (m_handeyeClient.saveCalibration())
  {
    m_ui->textEditRobotStatus->setText("Calibration saved");
    ROS_DEBUG("Could not save the calibration");
  }
  else
  {
    ROS_DEBUG("Could not save the calibration");
  }
  m_ui->saveButton->setEnabled(true);
  updateUI();
}

void CalibrationPanel::onCheckStartingPosePressed(bool) {
  m_canCalibrate = false;
  m_ui->checkStartPoseButton->setEnabled(false);
  m_ui->textEditRobotStatus->setText("Checking if the robot can rotate/translate in all directions...");
  QCoreApplication::processEvents();

  if (m_robotMovementsClient.checkStartingPose(m_targetPoses)) {
    m_ui->textEditRobotStatus->setText("The calibration can be performed from the current position");
    m_canCalibrate = true;
  } else {
    m_ui->textEditRobotStatus->setText("The calibration CANNOT be performed from the current position; move the robot so that it can rotate the end effector in all directions");
  }

  m_ui->checkStartPoseButton->setEnabled(true);
  updateUI();
}

void CalibrationPanel::setSampleList(const easy_handeye_msgs::SampleList& new_list, std::optional<size_t> focused_item_index)
{
  ROS_ASSERT(new_list.camera_marker_samples.size() == new_list.hand_world_samples.size());

  m_ui->sampleListWidget->clear();

  if (new_list.camera_marker_samples.empty())
    return;

  if (!focused_item_index.has_value())
    focused_item_index = 0;

  focused_item_index = std::min(*focused_item_index, new_list.camera_marker_samples.size()-1);

  ROS_ASSERT(focused_item_index < new_list.camera_marker_samples.size() && focused_item_index >= 0);


  for (int i = 0; i < new_list.camera_marker_samples.size(); i++)
  {
    auto formatSample = [](const geometry_msgs::Transform& s) {
      std::stringstream sample_stream;
      sample_stream << std::fixed << std::setprecision(3);
      sample_stream << "tr: [" << s.translation.x << ", " << s.translation.y << ", " << s.translation.z << "]"
                   << std::endl;
      sample_stream << "rot: [" << s.rotation.x << ", " << s.rotation.y << ", " << s.rotation.z << ", "
                   << s.rotation.w << "]";
      return sample_stream.str();
    };

    std::string formatted_hand_world_sample = formatSample(new_list.hand_world_samples[i]);
    std::string formatted_camera_marker_sample = formatSample(new_list.camera_marker_samples[i]);

    std::stringstream sample_stream;
    sample_stream << i << ") " << std::endl
                 << "hand->world:" << std::endl
                 << formatted_hand_world_sample << std::endl
                 << "camera->marker:" << std::endl
                 << formatted_camera_marker_sample;
    std::string sampleString = sample_stream.str();

    m_ui->sampleListWidget->addItem(QString::fromStdString(sampleString));
  }

  // this could have been the last sample
  if (focused_item_index < m_ui->sampleListWidget->count())
    m_ui->sampleListWidget->setCurrentRow(*focused_item_index);

  updateUI();
}


void CalibrationPanel::computeCalibration()
{
  // TODO: client->enoughSamples()
  if (m_ui->sampleListWidget->count() > 3)
  {
    easy_handeye_msgs::HandeyeCalibration result_calibration;
    if (m_handeyeClient.computeCalibration(result_calibration))
    {
      auto formatSample = [](const geometry_msgs::Transform& s) {
        std::stringstream sample_stream;
        sample_stream << std::fixed << std::setprecision(3);
        sample_stream << "tr: [" << s.translation.x << ", " << s.translation.y << ", " << s.translation.z << "]"
                      << std::endl;
        sample_stream << "rot: [" << s.rotation.x << ", " << s.rotation.y << ", " << s.rotation.z << ", "
                      << s.rotation.w << "]";
        return sample_stream.str();
      };

      std::stringstream s;
      s << formatSample(result_calibration.transform.transform) << std::endl << std::endl;
      s << "Outliers detected: TODO" << std::endl << std::endl;
      s << "The calibration could already be computed, but the more samples the better." << std::endl << std::endl;
      s << "Remember to SAVE the calibration!" << std::endl;
      m_ui->resultTextEdit->setPlainText(QString::fromStdString(s.str()));

      m_ui->saveButton->setEnabled(true);
      m_ui->saveButton->setFocus();
      return;
    } else {
      m_ui->resultTextEdit->setPlainText("Could not compute the calibration: communication with the backend failed");
    }
  } else {
    m_ui->resultTextEdit->setPlainText("Not enough samples to compute the calibration");
    ROS_DEBUG("Could not compute the calibration");
    // TODO: feedback to the user
  }
}

void CalibrationPanel::updateUI()
{
  m_ui->removeButton->setEnabled(m_ui->sampleListWidget->count() > 0);
  m_ui->saveButton->setEnabled(m_calibrationComputed);

  bool hasPoses = m_canCalibrate && m_targetPoses.has_value();
  int i = m_targetPoses->current_target_pose_index;
  int len = m_targetPoses->target_poses.size();
  bool hasPrevPose = hasPoses && i > 0;
  bool hasNextPose = hasPoses && i < len;
  bool canPlan = hasPoses && 0 < i && i < len;
  m_ui->pushButtonPosePrevious->setEnabled(hasPrevPose);
  m_ui->pushButtonPoseNext->setEnabled(hasNextPose);
  m_ui->nextPoseButton->setEnabled(hasNextPose);
  m_ui->planButton->setEnabled(canPlan);
  m_ui->moveButton->setEnabled(m_hasNewPlan);
}

}  // namespace easy_handeye_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(easy_handeye_rviz_plugins::CalibrationPanel, rviz::Panel)