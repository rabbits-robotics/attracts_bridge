#include <rclcpp/rclcpp.hpp>

#include "attracts_bridge/stm32_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stm32Bridge>());
  rclcpp::shutdown();
  return 0;
}
