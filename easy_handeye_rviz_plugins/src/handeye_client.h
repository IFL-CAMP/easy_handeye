
#pragma once

#include <ros/node_handle.h>
#include <easy_handeye_msgs/SampleList.h>
#include <easy_handeye_msgs/HandeyeCalibration.h>

namespace easy_handeye_rviz_plugins {

class HandeyeClient {
public:
  HandeyeClient();

  bool getSampleList(easy_handeye_msgs::SampleList& out);
  bool takeSample(easy_handeye_msgs::SampleList& newList);
  bool removeSample(size_t index, easy_handeye_msgs::SampleList& newList);
  bool computeCalibration(easy_handeye_msgs::HandeyeCalibration& result);
  bool saveCalibration();

private:

  bool is_eye_on_hand{};
  std::string robot_base_frame;
  std::string robot_effector_frame;
  std::string tracking_base_frame;
  std::string tracking_marker_frame;

  ros::ServiceClient get_sample_list_client;
  ros::ServiceClient take_sample_client;
  ros::ServiceClient remove_sample_client;
  ros::ServiceClient compute_calibration_client;
  ros::ServiceClient save_calibration_client;

  ros::NodeHandle nh;
};

};
