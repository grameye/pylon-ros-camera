#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <camera_control_msgs/SetIntegerValue.h>

namespace g3i_camera_initializer
{
class CameraInitializer
{
public:
  CameraInitializer();
  ~CameraInitializer();
  virtual void initNode(std::shared_ptr<ros::NodeHandle> nh);

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::ServiceClient stop_grabbing_client_;
  ros::ServiceClient max_transfer_size_client_;

  bool startup_grabbing_ = false;
  int max_transfer_size_ = 4194304;

  void initializeSetting();
  void initializeStartupGrabbing();
  void initializeMaxTransferSize();
};
} // namespace g3i_camera_initializer
