#include <g3i_camera_initializer/camera_initializer.h>

namespace g3i_camera_initializer
{
CameraInitializer::CameraInitializer()
{
}

CameraInitializer::~CameraInitializer()
{
}

void CameraInitializer::initNode(std::shared_ptr<ros::NodeHandle> nh)
{
  this->nh_ = nh;

  this->stop_grabbing_client_ = this->nh_->serviceClient<std_srvs::Trigger>(
    "pylon_camera_node/stop_grabbing");
  this->max_transfer_size_client_ = this->nh_->serviceClient<camera_control_msgs::SetIntegerValue>(
    "pylon_camera_node/set_max_transfer_size");
  
  this->nh_->getParam(
    "/g3i_camera_initializer_node/startup_grabbing", startup_grabbing_);
  this->nh_->getParam(
    "/g3i_camera_initializer_node/max_transfer_size", max_transfer_size_);

  ROS_INFO("[g3i_camera_initializer] startup_grabbing: %d", startup_grabbing_);
  ROS_INFO("[g3i_camera_initializer] max_transfer_size: %d", max_transfer_size_);

  initializeSetting();
}

/// @brief カメラ設定の書き換え
void CameraInitializer::initializeSetting()
{
  initializeStartupGrabbing();
  initializeMaxTransferSize();
  return;
}

/// @brief 起動時の撮像オンオフの設定
void CameraInitializer::initializeStartupGrabbing()
{
  ros::service::waitForService("/g3i_camera_initializer_node/startup_grabbing");

  // NOTE:起動時はデフォルトでオンになっている
  if (startup_grabbing_ == false)
  {
    std_srvs::Trigger trigger_srv_;
    // TODO:エラー番号を付与する
    if(!stop_grabbing_client_.call(trigger_srv_))
    {
      ROS_ERROR("[g3i_camera_initializer] Failed to set stop_grabbing.");
    }
  }

  return;
}

/// @brief 最大転送サイズの設定
void CameraInitializer::initializeMaxTransferSize()
{
  ros::service::waitForService("/g3i_camera_initializer_node/max_transfer_size");

  camera_control_msgs::SetIntegerValue integer_srv_;
  integer_srv_.request.value = max_transfer_size_;
  // TODO:エラー番号を付与する
  if(!max_transfer_size_client_.call(integer_srv_))
  {
    ROS_ERROR("[g3i_camera_initializer] Failed to set max_transfer_size.");
  }
  
  return;
}
}