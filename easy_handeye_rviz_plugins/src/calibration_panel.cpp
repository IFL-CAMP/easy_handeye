
#include <QLabel>
#include <QPainter>
#include <QTimer>

#include "calibration_panel.h"
#include </home/marco/ros/catkin_ws/build/easy_handeye_rviz_plugins/easy_handeye_rviz_plugins_autogen/include/ui_calibration_panel.h>  // TODO remove
//#include <ui_calibration_panel.h>

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
  if (client.getSampleList(sl))
    setSampleList(sl);

  connect(m_ui->comboBox_namespace, QOverload<const QString&>::of(&QComboBox::currentIndexChanged),
          [this](const QString& newNamespace) { activateCalibration(newNamespace.toStdString()); });

  connect(m_ui->takeButton, &QPushButton::clicked, this, &CalibrationPanel::onTakeSamplePressed);
  connect(m_ui->removeButton, &QPushButton::clicked, this, &CalibrationPanel::onRemoveSamplePressed);
  connect(m_ui->computeButton, &QPushButton::clicked, this, &CalibrationPanel::onComputeCalibrationPressed);
  connect(m_ui->saveButton, &QPushButton::clicked, this, &CalibrationPanel::onSaveCalibrationPressed);
}

void CalibrationPanel::activateCalibration(const std::string& calibrationNamespace)
{
  client.selectCalibration(calibrationNamespace);

  m_ui->calibNameLineEdit->setText(QString::fromStdString(calibrationNamespace));
  m_ui->calibTypeLineEdit->setText(client.isEyeOnHand() ? "eye on hand" : "eye on base");
  m_ui->robotBaseFrameLineEdit->setText(QString::fromStdString(client.robotBaseFrame()));
  m_ui->robotEffectorFrameLineEdit->setText(QString::fromStdString(client.robotEffectorFrame()));
  m_ui->trackingBaseFrameLineEdit->setText(QString::fromStdString(client.trackingBaseFrame()));
  m_ui->trackingMarkerFrameLineEdit->setText(QString::fromStdString(client.trackingMarkerFrame()));
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
  easy_handeye_msgs::SampleList new_sample_list;
  if (client.takeSample(new_sample_list))
  {
    setSampleList(new_sample_list);
  }
  else
  {
    ROS_DEBUG("Could not get a new sample");
  }
}

void CalibrationPanel::onRemoveSamplePressed(bool)
{
  int sample_index = m_ui->sampleListWidget->currentRow();
  easy_handeye_msgs::SampleList newSampleList;
  if (client.removeSample(sample_index, newSampleList))
  {
    setSampleList(newSampleList);
  }
  else
  {
    ROS_DEBUG_STREAM("Could not remove sample " << sample_index);
  }
}

void CalibrationPanel::onComputeCalibrationPressed(bool)
{
  easy_handeye_msgs::HandeyeCalibration result_calibration;
  if (client.computeCalibration(result_calibration))
  {
    setCalibrationTransform(result_calibration.transform);
  }
  else
  {
    ROS_DEBUG("Could not compute the calibration");
  }
}

void CalibrationPanel::onSaveCalibrationPressed(bool)
{
  if (client.saveCalibration())
  {
    onSavedCalibration();
  }
  else
  {
    ROS_DEBUG("Could not compute the calibration");
  }
}
void CalibrationPanel::setSampleList(const easy_handeye_msgs::SampleList& new_list, int focused_item_index)
{
  ROS_ASSERT(new_list.camera_marker_samples.size() == new_list.hand_world_samples.size());

  m_ui->sampleListWidget->clear();

  if (new_list.camera_marker_samples.empty())
    return;

  if (focused_item_index < 0)
    focused_item_index = new_list.camera_marker_samples.size() + focused_item_index;

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
    m_ui->sampleListWidget->setCurrentRow(focused_item_index);

  updateUI();
}

void CalibrationPanel::setCalibrationTransform(const geometry_msgs::TransformStamped& calibration_transform)
{
  // TODO: print the calibration transformation into its widget
  std::stringstream s;
  s << calibration_transform.transform;
  m_ui->resultTextEdit->setPlainText(QString::fromStdString(s.str()));
  updateUI();
}

void CalibrationPanel::onSavedCalibration()
{
  // TODO: give feedback to the user
  updateUI();
}

void CalibrationPanel::updateUI()
{
  m_ui->removeButton->setEnabled(m_ui->sampleListWidget->count() > 0);
  m_ui->computeButton->setEnabled(m_ui->sampleListWidget->count() > 3); // make dynamic per-algorithm?
  m_ui->saveButton->setEnabled(m_ui->computeButton->isEnabled()); // TODO: only if calibration has been computed
}

}  // namespace easy_handeye_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(easy_handeye_rviz_plugins::CalibrationPanel, rviz::Panel)