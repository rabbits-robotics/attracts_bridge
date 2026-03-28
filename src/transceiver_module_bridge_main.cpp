#include <rclcpp/rclcpp.hpp>

#include "attracts_bridge/transceiver_module_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransceiverModuleBridge>());
  rclcpp::shutdown();
  return 0;
}
