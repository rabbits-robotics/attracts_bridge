#ifndef ATTRACTS_BRIDGE__STM32_BRIDGE_NODE_HPP_
#define ATTRACTS_BRIDGE__STM32_BRIDGE_NODE_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <attracts_msgs/msg/attracts_command.hpp>

class Stm32Bridge : public rclcpp::Node
{
public:
  Stm32Bridge();

private:
  int OpenSerialPort(const std::string & device_name);
  void CmdCB(const attracts_msgs::msg::AttractsCommand::SharedPtr msg);
  void SendSerialData(const uint8_t buf[8]);

  rclcpp::Subscription<attracts_msgs::msg::AttractsCommand>::SharedPtr cmd_sub_;

  std::string device_name_;
  int fd1_ = -1;
};

#endif  // ATTRACTS_BRIDGE__STM32_BRIDGE_NODE_HPP_
