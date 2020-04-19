

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>

#include "calibration_panel.h"

namespace easy_handeye_rviz_plugins {

CalibrationPanel::CalibrationPanel(QWidget *parent) : rviz::Panel(parent) {
  auto *topic_layout = new QHBoxLayout();
  topic_layout->addWidget(new QLabel("Output Topic:"));
  output_topic_editor_ = new QLineEdit();
  topic_layout->addWidget(output_topic_editor_);

  auto *layout = new QVBoxLayout();
  layout->addLayout(topic_layout);
  setLayout(layout);
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void CalibrationPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  //  config.mapSetValue("Topic", output_topic_);
}

// Load all configuration data for this panel from the given Config object.
void CalibrationPanel::load(const rviz::Config &config) {
  rviz::Panel::load(config);
  //  QString topic;
  //  if (config.mapGetString("Topic", &topic)) {
  //  }
}

} // namespace easy_handeye_rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(easy_handeye_rviz_plugins::CalibrationPanel, rviz::Panel)