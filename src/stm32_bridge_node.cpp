#include "attracts_bridge/stm32_bridge_node.hpp"

#include <cerrno>
#include <cstring>
#include <functional>

#include <rabcl/interface/uart.hpp>
#include <rabcl_ros2/utils.hpp>

Stm32Bridge::Stm32Bridge()
: Node("stm32_bridge_node")
{
  device_name_ = "/dev/ttyACM0";
  fd1_ = OpenSerialPort(device_name_);
  if (fd1_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Serial Failed: could not open %s", device_name_.c_str());
    rclcpp::shutdown();
  }

  cmd_sub_ = this->create_subscription<attracts_msgs::msg::AttractsCommand>(
    "cmd", 10,
    std::bind(&Stm32Bridge::CmdCB, this, std::placeholders::_1));
}

int Stm32Bridge::OpenSerialPort(const std::string & device_name)
{
  int fd1 = open(device_name.c_str(), O_RDWR | O_NOCTTY);

  fcntl(fd1, F_SETFL, 0);
  struct termios conf_tio;
  tcgetattr(fd1, &conf_tio);

  speed_t baudrate = B115200;
  cfsetispeed(&conf_tio, baudrate);
  cfsetospeed(&conf_tio, baudrate);

  conf_tio.c_cflag = CS8 | CLOCAL | CREAD | B115200;
  conf_tio.c_iflag = IGNPAR;
  conf_tio.c_lflag &= ~(ECHO | ICANON);

  conf_tio.c_cc[VMIN] = 1;
  conf_tio.c_cc[VTIME] = 1;

  tcflush(fd1, TCIOFLUSH);
  tcsetattr(fd1, TCSANOW, &conf_tio);

  return fd1;
}

void Stm32Bridge::CmdCB(const attracts_msgs::msg::AttractsCommand::SharedPtr msg)
{
  rabcl::Info info;
  rabcl_ros2::Utils::CmdMsgToInfo(*msg, info);

  rabcl::Uart uart;
  uart.PreparePacket(info);
  SendSerialData(uart.uart_transmit_buffer_);
}

void Stm32Bridge::SendSerialData(const uint8_t * buf)
{
  int rec = write(fd1_, buf, rabcl::Uart::PACKET_SIZE);
  if (rec < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write: %s", strerror(errno));
  } else if (rec != rabcl::Uart::PACKET_SIZE) {
    RCLCPP_WARN(this->get_logger(), "Serial Warning: only %d bytes written", rec);
  } else {
    tcdrain(fd1_);
  }
}

#ifndef ATTRACTS_BRIDGE_EXCLUDE_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Stm32Bridge>());
  rclcpp::shutdown();
  return 0;
}
#endif
