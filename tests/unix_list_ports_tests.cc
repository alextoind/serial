#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <serial/serial.h>

using namespace serial;

namespace {

#if defined(__linux__)
TEST(ListSerialPorts, PortsList) {
  std::vector<std::string> serial_port_names;
  int serial_port_names_retrieved = getPortsList(serial_port_names);
  ASSERT_GE(serial_port_names_retrieved, 0);
  ASSERT_EQ(serial_port_names.size(), serial_port_names_retrieved);
  for (auto const &serial_port_name : serial_port_names) {
    EXPECT_THAT(serial_port_name, ::testing::MatchesRegex("^/dev/([^/]+)/?$"));
  }
}

TEST(ListSerialPorts, PortsInfo) {
  std::vector<PortInfo> serial_ports;
  int serial_ports_retrieved = getPortsInfo(serial_ports);
  ASSERT_GE(serial_ports_retrieved, 0);
  ASSERT_EQ(serial_ports.size(), serial_ports_retrieved);
  for (auto const &serial_port : serial_ports) {
    ASSERT_THAT(serial_port.serial_port, ::testing::MatchesRegex("^/dev/([^/]+)/?$"));
  }
}

TEST(ListSerialPorts, PortInfo) {
  PortInfo serial_port;
  if (!serial_port.getPortInfo("/dev/ttyUSB0")) {  //TODO: this should be device agnostic
    EXPECT_GE(serial_port.busnum, 0);
    EXPECT_LE(serial_port.busnum, 65535);
    EXPECT_GE(serial_port.devnum, 0);
    EXPECT_LE(serial_port.devnum, 65535);
    EXPECT_THAT(std::to_string(serial_port.id_product), ::testing::MatchesRegex("[0-9a-fA-F]{1,4}"));
    EXPECT_THAT(std::to_string(serial_port.id_vendor), ::testing::MatchesRegex("[0-9a-fA-F]{1,4}"));
    EXPECT_EQ(serial_port.manufacturer, "QB Robotics");
    EXPECT_THAT(serial_port.product, ::testing::MatchesRegex("[A-Z]+ [0-9]+"));
    EXPECT_THAT(serial_port.product, ::testing::EndsWith(serial_port.serial_number));
    EXPECT_THAT(serial_port.serial_number, ::testing::MatchesRegex("[0-9]+"));
    ASSERT_EQ(serial_port.serial_port, "/dev/ttyUSB0");
  } else {
    ASSERT_EQ(serial_port.busnum, 0);
    ASSERT_EQ(serial_port.devnum, 0);
    ASSERT_EQ(serial_port.id_product, 0);
    ASSERT_EQ(serial_port.id_vendor, 0);
    ASSERT_EQ(serial_port.manufacturer, "");
    ASSERT_EQ(serial_port.product, "");
    ASSERT_EQ(serial_port.serial_number, "");
    ASSERT_EQ(serial_port.serial_port, "");
  }
}
#endif

TEST(timer_tests, dummy) {
  ASSERT_TRUE(true);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
