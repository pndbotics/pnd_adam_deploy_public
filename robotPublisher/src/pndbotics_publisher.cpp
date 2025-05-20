#include "pndbotics_publisher.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPublisher>());
  rclcpp::shutdown();
  return 0;
}