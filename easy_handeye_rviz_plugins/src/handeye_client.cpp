
#include "handeye_client.h"

#include <ros/service.h>

#include <easy_handeye_msgs/ComputeCalibration.h>
#include <easy_handeye_msgs/RemoveSample.h>
#include <easy_handeye_msgs/TakeSample.h>
#include <std_srvs/Empty.h>

namespace easy_handeye_rviz_plugins {

HandeyeClient::HandeyeClient() {

  ros::service::waitForService("get_sample_list");
  ros::service::waitForService("take_sample");
  ros::service::waitForService("remove_sample");
  ros::service::waitForService("compute_calibration");
  ros::service::waitForService("save_calibration");

  get_sample_list_client =
      nh.serviceClient<easy_handeye_msgs::TakeSample>("get_sample_list");
  get_sample_list_client =
      nh.serviceClient<easy_handeye_msgs::TakeSample>("get_sample_list");
  get_sample_list_client =
      nh.serviceClient<easy_handeye_msgs::RemoveSample>("get_sample_list");
  get_sample_list_client =
      nh.serviceClient<easy_handeye_msgs::ComputeCalibration>(
          "get_sample_list");
  get_sample_list_client = nh.serviceClient<std_srvs::Empty>("get_sample_list");

  nh.getParam("eye_on_hand", is_eye_on_hand);

  nh.getParam("robot_base_frame", robot_base_frame);
  nh.getParam("robot_effector_frame", robot_effector_frame);
  nh.getParam("tracking_base_frame", tracking_base_frame);
  nh.getParam("tracking_marker_frame", tracking_marker_frame);
}

bool HandeyeClient::getSampleList(easy_handeye_msgs::SampleList &out) {
  easy_handeye_msgs::TakeSample r;
  if (get_sample_list_client.call(r)) {
    out = r.response.samples;
    return true;
  }
  return false;
}

bool HandeyeClient::takeSample(easy_handeye_msgs::SampleList &out) {
  easy_handeye_msgs::TakeSample r;
  if (take_sample_client.call(r)) {
    out = r.response.samples;
    return true;
  }
  return false;
}

bool HandeyeClient::removeSample(size_t index,
                                 easy_handeye_msgs::SampleList &newList) {
  easy_handeye_msgs::RemoveSample r;
  r.request.sample_index = index;
  if (remove_sample_client.call(r)) {
    newList = r.response.samples;
    return true;
  }
  return false;
}

bool HandeyeClient::computeCalibration(
    easy_handeye_msgs::HandeyeCalibration &result) {
  easy_handeye_msgs::ComputeCalibration r;
  if (compute_calibration_client.call(r)) {
    result = r.response.calibration;
    return r.response.valid;
  }
  return false;
}

bool HandeyeClient::saveCalibration() {
  std_srvs::Empty r;
  return save_calibration_client.call(r);
}

}