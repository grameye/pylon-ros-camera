#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <camera_control_msgs/SetIntegerValue.h>
#include <camera_control_msgs/SetSleeping.h>

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
  ros::ServiceClient light_source_preset_client_;
  ros::ServiceClient set_sleeping_client_;

  bool startup_grabbing_ = false;
  int max_transfer_size_ = 4194304;
  int light_source_preset_ = 6;
  bool startup_sleeping_ = false;

  void initializeSetting();
  void initializeStartupGrabbing();
  void initializeMaxTransferSize();
  void initializeLightSourcePreset();
  void initializeStartupSleeping();
};
} // namespace g3i_camera_initializer
