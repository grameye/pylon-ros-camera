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
  this->light_source_preset_client_ = this->nh_->serviceClient<camera_control_msgs::SetIntegerValue>(
    "pylon_camera_node/set_light_source_preset");
  this->set_sleeping_client_ = this->nh_->serviceClient<camera_control_msgs::SetSleeping>(
    "pylon_camera_node/set_sleeping");

  this->nh_->getParam(
    "/g3i_camera_initializer_node/startup_grabbing", startup_grabbing_);
  this->nh_->getParam(
    "/g3i_camera_initializer_node/max_transfer_size", max_transfer_size_);
  this->nh_->getParam(
    "/g3i_camera_initializer_node/light_source_preset", light_source_preset_);
  this->nh_->getParam(
    "/g3i_camera_initializer_node/startup_sleeping", startup_sleeping_);

  ROS_INFO("[g3i_camera_initializer] startup_grabbing: %d", startup_grabbing_);
  ROS_INFO("[g3i_camera_initializer] max_transfer_size: %d", max_transfer_size_);
  ROS_INFO("[g3i_camera_initializer] light_source_preset: %d", light_source_preset_);
  ROS_INFO("[g3i_camera_initializer] startup_sleeping: %d", startup_sleeping_);

  initializeSetting();
}

/// @brief カメラ設定の書き換え
void CameraInitializer::initializeSetting()
{
  // NOTE:max_transfer_sizeを変更すると自動でカメラオンになってしまう
  initializeMaxTransferSize();
  initializeLightSourcePreset();
  initializeStartupGrabbing();
  initializeStartupSleeping();
}

/// @brief 起動時の撮像オンオフの設定
void CameraInitializer::initializeStartupGrabbing()
{
  ros::service::waitForService("/pylon_camera_node/stop_grabbing");

  // NOTE:起動時はデフォルトでオンになっている
  if (startup_grabbing_ == false) {
    std_srvs::Trigger trigger_srv_;
    // TODO:エラー番号を付与する
    if (stop_grabbing_client_.call(trigger_srv_)) {
      ROS_INFO("[g3i_camera_initializer] Successed to set stop_grabbing.");
    } else {
      ROS_ERROR("[g3i_camera_initializer] Failed to set stop_grabbing.");
    }
  }

}

/// @brief 最大転送サイズの設定
void CameraInitializer::initializeMaxTransferSize()
{
  ros::service::waitForService("/pylon_camera_node/set_max_transfer_size");

  camera_control_msgs::SetIntegerValue integer_srv_;
  integer_srv_.request.value = max_transfer_size_;
  // TODO:エラー番号を付与する
  if (max_transfer_size_client_.call(integer_srv_)) {
    ROS_INFO("[g3i_camera_initializer] Successed to set max_transfer_size.");
  } else {
    ROS_ERROR("[g3i_camera_initializer] Failed to set max_transfer_size.");
  }
}

/// @brief LightSourcePresetの設定(明るさ)
void CameraInitializer::initializeLightSourcePreset()
{
  ros::service::waitForService("/pylon_camera_node/set_light_source_preset");

  camera_control_msgs::SetIntegerValue integer_srv_;
  integer_srv_.request.value = light_source_preset_;
  // TODO:エラー番号を付与する
  if (light_source_preset_client_.call(integer_srv_)) {
    ROS_INFO("[g3i_camera_initializer] Successed to set light_source_preset.");
  } else {
    ROS_ERROR("[g3i_camera_initializer] Failed to set light_source_preset.");
  }
}

/// @brief 起動時の休止状態の設定
void CameraInitializer::initializeStartupSleeping()
{
  ros::service::waitForService("/pylon_camera_node/set_sleeping");

  // NOTE:起動時はデフォルトでオンになっている
  camera_control_msgs::SetSleeping set_sleeping_srv_;
  set_sleeping_srv_.req.set_sleeping = startup_sleeping;
  // TODO:エラー番号を付与する
  if (set_sleeping_client_.call(set_sleeping_srv_)) {
    ROS_INFO("[g3i_camera_initializer] Successed to set set_sleeping.");
  } else {
    ROS_ERROR("[g3i_camera_initializer] Failed to set set_sleeping.");
  }

}
}
