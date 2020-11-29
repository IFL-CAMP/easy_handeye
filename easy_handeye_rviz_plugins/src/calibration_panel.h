
#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <optional>

#include <rviz/panel.h>
#include <easy_handeye_msgs/TargetPoseList.h>
#endif

#include "handeye_client.h"
#include "robot_movements_client.h"

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
  // sampling
  void onTakeSamplePressed(bool);
  void onRemoveSamplePressed(bool);
  // calibration
  void onSaveCalibrationPressed(bool);
  // robot motion
  void onCheckStartingPosePressed(bool);
  void onPoseNextPressed(bool);
  void onPosePreviousPressed(bool);
  void onHomeAndNextPosePressed(bool);
  void onPlanPressed(bool);
  void onMovePressed(bool);

protected:
  void activateCalibration(const std::string& calibrationNamespace);
  void updateUI();

  void setSampleList(const easy_handeye_msgs::SampleList& new_list, std::optional<size_t> focused_item_index = -1);
  void computeCalibration();

  Ui_CalibrationPanel* m_ui;

  bool m_canCalibrate = false;
  bool m_hasNewPlan = false;
  std::optional<easy_handeye_msgs::TargetPoseList> m_targetPoses = {};
  bool m_calibrationComputed = false;

  HandeyeClient m_handeyeClient;
  RobotMovementsClient m_robotMovementsClient;
};

}  // namespace easy_handeye_rviz_plugins
