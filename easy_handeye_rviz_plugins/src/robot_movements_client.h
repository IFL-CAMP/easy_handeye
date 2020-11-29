
#pragma once

#include <ros/node_handle.h>
#include <easy_handeye_msgs/SampleList.h>
#include <easy_handeye_msgs/HandeyeCalibration.h>
#include <easy_handeye_msgs/TargetPoseList.h>

namespace easy_handeye_rviz_plugins
{
class RobotMovementsClient
{
public:
  RobotMovementsClient();

  void selectCalibration(const std::string& calibration_namespace);

  bool init(int32_t timeout_seconds);

  bool checkStartingPose(std::optional<easy_handeye_msgs::TargetPoseList>& targetPoses);

private:

  std::string active_calibration_namespace;

  bool is_ready = false;

  ros::ServiceClient check_starting_pose_client;
  ros::ServiceClient enumerate_target_poses_client;
  ros::ServiceClient select_target_pose_client;
  ros::ServiceClient plan_to_target_pose_client;
  ros::ServiceClient execute_plan_client;

  ros::NodeHandle nh;
};

}  // namespace easy_handeye_rviz_plugins
