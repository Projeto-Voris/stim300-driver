#include "rclcpp/rclcpp.hpp"
#include "ros_stim300_driver/ros_stim300_driver.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stim300DriverNode>());
  rclcpp::shutdown();
  return 0;
}