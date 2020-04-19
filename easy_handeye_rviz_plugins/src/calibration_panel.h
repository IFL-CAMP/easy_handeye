
#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

class QLineEdit;

namespace easy_handeye_rviz_plugins {

class CalibrationPanel : public rviz::Panel {
  Q_OBJECT
public:
  explicit CalibrationPanel(QWidget *parent = nullptr);
  ~CalibrationPanel() override = default;

  void load(const rviz::Config &config) override;
  void save(rviz::Config config) const override;

public Q_SLOTS:

protected Q_SLOTS:

protected:

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit *output_topic_editor_;
  ros::NodeHandle nh_;
};

}
