
#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include "handeye_client.h"

class Ui_CalibrationPanel;

namespace easy_handeye_rviz_plugins
{
class CalibrationPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit CalibrationPanel(QWidget* parent = nullptr);
  ~CalibrationPanel() override = default;

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

public Q_SLOTS:

protected Q_SLOTS:
  void onTakeSamplePressed(bool);
  void onRemoveSamplePressed(bool);
  void onSaveCalibrationPressed(bool);

protected:
  void activateCalibration(const std::string& calibrationNamespace);
  void updateUI();

  void setSampleList(const easy_handeye_msgs::SampleList& new_list, int focused_item_index = -1);
  void onSavedCalibration();

  Ui_CalibrationPanel* m_ui;

  HandeyeClient client;
};

}  // namespace easy_handeye_rviz_plugins
