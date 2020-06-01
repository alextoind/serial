#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <serial/serial.h>

using namespace serial;

namespace {

TEST(ListSerialPorts, PortInfoQB) {
  PortInfo serial_port;
  serial_port.getPortInfo("/dev/ttyUSB0");

  EXPECT_GE(serial_port.busnum, 0);
  EXPECT_LE(serial_port.busnum, 1024);
  EXPECT_GE(serial_port.devnum, 0);
  EXPECT_LE(serial_port.devnum, 1024);
  EXPECT_THAT(std::to_string(serial_port.id_product), ::testing::MatchesRegex("[0-9]{1,4}"));
  EXPECT_THAT(std::to_string(serial_port.id_vendor), ::testing::MatchesRegex("[0-9]{1,4}"));
  EXPECT_EQ(serial_port.manufacturer, "QB Robotics");
  EXPECT_THAT(serial_port.product, ::testing::EndsWith(serial_port.serial_number));
  EXPECT_THAT(serial_port.serial_number, ::testing::MatchesRegex("[0-9]+"));
  ASSERT_EQ(serial_port.serial_port, "/dev/ttyUSB0");
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
