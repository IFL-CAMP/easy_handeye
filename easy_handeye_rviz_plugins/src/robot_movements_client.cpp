
#include "robot_movements_client.h"
#include "utils.h"

#include <ros/master.h>
#include <ros/service.h>

#include <easy_handeye_msgs/CheckStartingPose.h>
#include <std_srvs/Empty.h>


namespace easy_handeye_rviz_plugins
{
RobotMovementsClient::RobotMovementsClient()
{
  init(1);
}

void RobotMovementsClient::selectCalibration(const std::string& calibration_namespace)
{
  active_calibration_namespace = calibration_namespace;
  is_ready = false;
  init(1);
}

bool RobotMovementsClient::init(int32_t timeout_seconds)
{
  if (active_calibration_namespace.empty())
  {
    ROS_WARN("No calibration specified, can't init RobotMovementsClient");
    is_ready = false;
    return false;
  }

  if (is_ready)
  {
    ROS_WARN_STREAM("already inited for calibration: " << active_calibration_namespace << ", skipping new init");
    return true;
  }

  const std::string& acn = active_calibration_namespace;

  for (const auto& service : { "check_starting_pose" })
  {
    if (!ros::service::waitForService(acn + service, timeout_seconds))
    {
      ROS_ERROR_STREAM("could not find service " << service);
      return false;
    }
  }

  check_starting_pose_client = nh.serviceClient<easy_handeye_msgs::CheckStartingPose>(acn + "check_starting_pose");

  is_ready = true;

  ROS_INFO_STREAM("ready for calibration: " << active_calibration_namespace);

  return true;
}

bool RobotMovementsClient::checkStartingPose(std::optional<easy_handeye_msgs::TargetPoseList>& targetPoses)
{
  ROS_INFO_STREAM("Checking starting pose");
  easy_handeye_msgs::CheckStartingPose r;
  if (check_starting_pose_client.call(r))
  {
    if (r.response.can_calibrate)
    {
      ROS_INFO_STREAM("Starting pose good, can proceed with calibration");
      targetPoses = r.response.target_poses;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Can't start calibration movements from here: the robot can't translate/rotate the end effector "
                       "in at least one direction");
      targetPoses = {};
    }
  }
  return false;
}

}  // namespace easy_handeye_rviz_plugins