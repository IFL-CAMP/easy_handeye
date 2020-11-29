
#include "handeye_client.h"
#include "utils.h"

#include <ros/master.h>
#include <ros/service.h>

#include <easy_handeye_msgs/ComputeCalibration.h>
#include <easy_handeye_msgs/RemoveSample.h>
#include <easy_handeye_msgs/TakeSample.h>
#include <std_srvs/Empty.h>


namespace easy_handeye_rviz_plugins
{
HandeyeClient::HandeyeClient()
{
  init(1);
}

std::vector<std::string> HandeyeClient::listRunningCalibrations()
{
  // list running services, find the HandEyeServer
  std::vector<std::string> ret;

  std::vector<std::string> services = listServices();

  for (const auto& service : services)
  {
    std::string topic_ending = "take_sample";

    if (hasEnding(service, topic_ending))
    {
      ret.push_back(service.substr(0, service.length() - topic_ending.length()));
    }
  }

  return ret;
}

void HandeyeClient::selectCalibration(const std::string& calibration_namespace)
{
  active_calibration_namespace = calibration_namespace;
  is_ready = false;
  init(1);
}

bool HandeyeClient::init(int32_t timeout_seconds)
{
  if (active_calibration_namespace.empty())
  {
    ROS_WARN("No calibration specified, can't init HandeyeClient");
    is_ready = false;
    return false;
  }

  if (is_ready)
  {
    ROS_WARN_STREAM("already inited for calibration: " << active_calibration_namespace << ", skipping new init");
    return true;
  }

  const std::string& acn = active_calibration_namespace;

  nh.getParam(acn + "/eye_on_hand", is_eye_on_hand);

  nh.getParam(acn + "/robot_base_frame", robot_base_frame);
  nh.getParam(acn + "/robot_effector_frame", robot_effector_frame);
  nh.getParam(acn + "/tracking_base_frame", tracking_base_frame);
  nh.getParam(acn + "/tracking_marker_frame", tracking_marker_frame);

  for (const auto& service :
       { "get_sample_list", "take_sample", "remove_sample", "compute_calibration", "save_calibration" })
  {
    if (!ros::service::waitForService(acn + service, timeout_seconds))
    {
      ROS_ERROR_STREAM("could not find service " << service);
      return false;
    }
  }

  get_sample_list_client = nh.serviceClient<easy_handeye_msgs::TakeSample>(acn + "get_sample_list");
  take_sample_client = nh.serviceClient<easy_handeye_msgs::TakeSample>(acn + "take_sample");
  remove_sample_client = nh.serviceClient<easy_handeye_msgs::RemoveSample>(acn + "remove_sample");
  compute_calibration_client = nh.serviceClient<easy_handeye_msgs::ComputeCalibration>(acn + "compute_calibration");
  save_calibration_client = nh.serviceClient<std_srvs::Empty>(acn + "save_calibration");

  is_ready = true;

  ROS_INFO_STREAM("ready for calibration: " << active_calibration_namespace);

  return true;
}

bool HandeyeClient::getSampleList(easy_handeye_msgs::SampleList& out)
{
  easy_handeye_msgs::TakeSample r;
  if (get_sample_list_client.call(r))
  {
    out = r.response.samples;
    return true;
  }
  return false;
}

bool HandeyeClient::takeSample(easy_handeye_msgs::SampleList& new_list)
{
  easy_handeye_msgs::TakeSample r;
  if (take_sample_client.call(r))
  {
    new_list = r.response.samples;
    return true;
  }
  return false;
}

bool HandeyeClient::removeSample(size_t index, easy_handeye_msgs::SampleList& new_list)
{
  easy_handeye_msgs::RemoveSample r;
  r.request.sample_index = index;
  if (remove_sample_client.call(r))
  {
    new_list = r.response.samples;
    return true;
  }
  return false;
}

bool HandeyeClient::computeCalibration(easy_handeye_msgs::HandeyeCalibration& result)
{
  easy_handeye_msgs::ComputeCalibration r;
  if (compute_calibration_client.call(r))
  {
    result = r.response.calibration;
    return r.response.valid;
  }
  return false;
}

bool HandeyeClient::saveCalibration()
{
  std_srvs::Empty r;
  return save_calibration_client.call(r);
}

}  // namespace easy_handeye_rviz_plugins