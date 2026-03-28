#include <gtest/gtest.h>

#include "attracts_bridge/transceiver_module_bridge_node.hpp"

// Helper to call ProcessFrameData with fresh output structs
static attracts_msgs::msg::GameDataInput ProcessInput(uint8_t * data, uint16_t len)
{
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 42;
  TransceiverModuleBridge::ProcessFrameData(0x00, data, len, input, robot, counter);
  return input;
}

static attracts_msgs::msg::GameDataRobot ProcessRobot(uint8_t * data, uint16_t len)
{
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 0;
  TransceiverModuleBridge::ProcessFrameData(0x01, data, len, input, robot, counter);
  return robot;
}

// struct DataInput (packed): int16 x, int16 y, uint8 buttons, kb1, kb2, kb3, kb4 = 9 bytes
// data_length check in ProcessFrameData is == 5 (matches the serial protocol header)

TEST(ProcessFrameDataTest, InputMouseDelta)
{
  // mouse_delta_x = 0x0102 = 258, mouse_delta_y = 0xFF00 = -256 (int16 little-endian)
  uint8_t data[9] = {0x02, 0x01, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
  auto out = ProcessInput(data, 5);
  EXPECT_EQ(258, out.mouse_delta_x);
  EXPECT_EQ(-256, out.mouse_delta_y);
}

TEST(ProcessFrameDataTest, InputMouseButtonsAllSet)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00};
  auto out = ProcessInput(data, 5);
  EXPECT_TRUE(out.mouse_left_button);
  EXPECT_TRUE(out.mouse_right_button);
  EXPECT_TRUE(out.mouse_wheel_button);
  EXPECT_TRUE(out.mouse_side_1_button);
  EXPECT_TRUE(out.mouse_side_2_button);
}

TEST(ProcessFrameDataTest, InputMouseButtonsNone)
{
  uint8_t data[9] = {};
  auto out = ProcessInput(data, 5);
  EXPECT_FALSE(out.mouse_left_button);
  EXPECT_FALSE(out.mouse_right_button);
  EXPECT_FALSE(out.mouse_wheel_button);
  EXPECT_FALSE(out.mouse_side_1_button);
  EXPECT_FALSE(out.mouse_side_2_button);
}

TEST(ProcessFrameDataTest, InputKeyboard1)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
  auto out = ProcessInput(data, 5);
  EXPECT_TRUE(out.key_1);
  EXPECT_TRUE(out.key_2);
  EXPECT_TRUE(out.key_3);
  EXPECT_TRUE(out.key_4);
  EXPECT_TRUE(out.key_5);
  EXPECT_TRUE(out.key_tab);
  EXPECT_TRUE(out.key_q);
  EXPECT_TRUE(out.key_w);
}

TEST(ProcessFrameDataTest, InputKeyboard2)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00};
  auto out = ProcessInput(data, 5);
  EXPECT_TRUE(out.key_e);
  EXPECT_TRUE(out.key_r);
  EXPECT_TRUE(out.key_t);
  EXPECT_TRUE(out.key_a);
  EXPECT_TRUE(out.key_s);
  EXPECT_TRUE(out.key_d);
  EXPECT_TRUE(out.key_f);
  EXPECT_TRUE(out.key_g);
}

TEST(ProcessFrameDataTest, InputKeyboard3)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00};
  auto out = ProcessInput(data, 5);
  EXPECT_TRUE(out.key_h);
  EXPECT_TRUE(out.key_shift);
  EXPECT_TRUE(out.key_z);
  EXPECT_TRUE(out.key_x);
  EXPECT_TRUE(out.key_c);
  EXPECT_TRUE(out.key_v);
  EXPECT_TRUE(out.key_b);
}

TEST(ProcessFrameDataTest, InputKeyboard4)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F};
  auto out = ProcessInput(data, 5);
  EXPECT_TRUE(out.key_ctrl);
  EXPECT_TRUE(out.key_alt);
  EXPECT_TRUE(out.key_space);
  EXPECT_TRUE(out.key_enter);
}

TEST(ProcessFrameDataTest, InputResetsControlLostCounter)
{
  uint8_t data[9] = {};
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 999;
  TransceiverModuleBridge::ProcessFrameData(0x00, data, 5, input, robot, counter);
  EXPECT_EQ(0, counter);
}

TEST(ProcessFrameDataTest, InputWrongLengthSkipped)
{
  uint8_t data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
  int counter = 42;
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  TransceiverModuleBridge::ProcessFrameData(0x00, data, 9, input, robot, counter);
  // data_length != 5 so not processed — counter unchanged
  EXPECT_EQ(42, counter);
}

// struct DataRobot (packed): uint8 type, team, speed_max, uint16 max_hp, cur_hp, max_heat, cur_heat
// = 3 + 8 = 11 bytes

TEST(ProcessFrameDataTest, RobotDataAllFields)
{
  // type=1, team=0, speed_max=30, max_hp=0x0400=1024, cur_hp=0x0300=768,
  // max_heat=0x0500=1280, cur_heat=0x0200=512
  uint8_t data[11] = {0x01, 0x00, 0x1E, 0x00, 0x04, 0x00, 0x03, 0x00, 0x05, 0x00, 0x02};
  auto out = ProcessRobot(data, 11);
  EXPECT_EQ(1, out.type);
  EXPECT_EQ(0, out.team);
  EXPECT_EQ(30, out.projectile_speed_max);
  EXPECT_EQ(1024, out.max_hp);
  EXPECT_EQ(768, out.current_hp);
  EXPECT_EQ(1280, out.max_heat);
  EXPECT_EQ(512, out.current_heat);
}

TEST(ProcessFrameDataTest, RobotDataWrongLengthSkipped)
{
  uint8_t data[11] = {0x01, 0x00, 0x1E, 0x00, 0x04, 0x00, 0x03, 0x00, 0x05, 0x00, 0x02};
  auto out = ProcessRobot(data, 10);
  // data_length != 11 so not processed — fields remain default (0)
  EXPECT_EQ(0, out.type);
}

TEST(ProcessFrameDataTest, UnknownCommandTypeNoOp)
{
  uint8_t data[1] = {0x00};
  attracts_msgs::msg::GameDataInput input;
  attracts_msgs::msg::GameDataRobot robot;
  int counter = 7;
  TransceiverModuleBridge::ProcessFrameData(0xFF, data, 1, input, robot, counter);
  EXPECT_EQ(7, counter);  // unchanged
}
