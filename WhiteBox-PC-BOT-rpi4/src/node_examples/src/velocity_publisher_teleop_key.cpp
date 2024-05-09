#include "velocity_publisher_teleop_key.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Spin until ROS is shutdown
  rclcpp::spin(std::make_shared<CmdVelPublisher_teleop_key>());

  rclcpp::shutdown();

  return 0;
}
