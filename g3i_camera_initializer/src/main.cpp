#include <ros/ros.h>
#include <g3i_camera_initializer/camera_initializer.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "g3i_camera_initializer_node");
  std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
  g3i_camera_initializer::CameraInitializer camera_initializer;
  camera_initializer.initNode(nh);
  ros::spin();
  return 0;
}
