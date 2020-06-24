/***
 *  MIT License
 *
 *  Copyright (c) 2020 Alessandro Tondo
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 *  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 *  to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 *  the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 *  THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

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
