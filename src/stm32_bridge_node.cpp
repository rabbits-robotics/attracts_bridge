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
  uart.PrepareFloatData(static_cast<uint8_t>(rabcl::UART_ID::UART_CHASSIS_X), info.chassis_vel_x_);
  SendSerialData(uart.uart_transmit_buffer_);
  uart.PrepareFloatData(static_cast<uint8_t>(rabcl::UART_ID::UART_CHASSIS_Y), info.chassis_vel_y_);
  SendSerialData(uart.uart_transmit_buffer_);
  uart.PrepareFloatData(static_cast<uint8_t>(rabcl::UART_ID::UART_CHASSIS_Z), info.chassis_vel_z_);
  SendSerialData(uart.uart_transmit_buffer_);
  uart.PrepareFloatData(static_cast<uint8_t>(rabcl::UART_ID::UART_YAW), info.yaw_pos_);
  SendSerialData(uart.uart_transmit_buffer_);
  uart.PrepareFloatData(static_cast<uint8_t>(rabcl::UART_ID::UART_PITCH), info.pitch_pos_);
  SendSerialData(uart.uart_transmit_buffer_);
  uint8_t mode_data[4] = {info.load_mode_, info.fire_mode_, info.speed_mode_, info.chassis_mode_};
  uart.Prepare4IntData(static_cast<uint8_t>(rabcl::UART_ID::UART_MODES), mode_data);
  SendSerialData(uart.uart_transmit_buffer_);
}

void Stm32Bridge::SendSerialData(const uint8_t buf[8])
{
  // check serial data
  RCLCPP_INFO(this->get_logger(), "serial_send:");
  for (int i = 0; i < 8; i++) {
    RCLCPP_INFO(this->get_logger(), "%x ", buf[i]);
  }

  // send data
  int rec = write(fd1_, buf, 8);
  if (rec < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write: %s", strerror(errno));
  } else if (rec != 8) {
    RCLCPP_WARN(this->get_logger(), "Serial Warning: only %d bytes written", rec);
  } else {
    RCLCPP_INFO(this->get_logger(), "Serial Write Successful (8 bytes)");
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
