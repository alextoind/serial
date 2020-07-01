/***
 *  MIT License
 *
 *  Copyright (c) 2020 Alessandro Tondo
 *  Copyright (c) 2012 William Woodall, John Harrison
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

#if defined(__linux__)
#include <pty.h>
#elif defined(__APPLE__)
#include <util.h>
#endif

using namespace serial;

namespace {

#if !defined(_WIN32)
class SerialTests : public ::testing::Test {
 protected:
  void SetUp() override {
    char pty_name[32] = "";
    ASSERT_TRUE(::openpty(&master_fd_, &slave_fd_, pty_name, nullptr, nullptr) != -1);
    ASSERT_TRUE(master_fd_ > 0);
    ASSERT_TRUE(slave_fd_ > 0);
    EXPECT_THAT(std::string(pty_name), ::testing::MatchesRegex("^/dev/.*"));
    EXPECT_NO_THROW(serial_port_ = std::unique_ptr<Serial>(new Serial(std::string(pty_name), 9600, Serial::Timeout(250))));
  }

  void TearDown() override {
    ::close(master_fd_);
    ::close(slave_fd_);
  }

  std::unique_ptr<Serial> serial_port_;
  int master_fd_ {0};
  int slave_fd_ {0};
};

TEST_F(SerialTests, OpenAndClose) {
  // cannot use EXPECT_* because on failure the following would be influenced
  ASSERT_TRUE(serial_port_->isOpen());
  ASSERT_THROW(serial_port_->open(), SerialException);  // already open
  ASSERT_TRUE(serial_port_->isOpen());
  ASSERT_NO_THROW(serial_port_->close());
  ASSERT_FALSE(serial_port_->isOpen());
  ASSERT_NO_THROW(serial_port_->open());
  ASSERT_TRUE(serial_port_->isOpen());
}

TEST_F(SerialTests, Reads) {
  ::write(master_fd_, "abcd1234", 8);
  EXPECT_EQ(serial_port_->read(4), std::string("abcd"));
  EXPECT_EQ(serial_port_->read(3), std::string("123"));
  EXPECT_EQ(serial_port_->read(2), std::string("4"));  // timeout

  std::string str;
  ::write(master_fd_, "efgh5678", 8);
  EXPECT_EQ(serial_port_->read(str, 4), 4);
  EXPECT_EQ(str, std::string("efgh"));
  EXPECT_EQ(serial_port_->read(str, 0), 0);
  EXPECT_EQ(str, std::string("efgh"));
  EXPECT_EQ(serial_port_->read(str, 3), 3);
  EXPECT_EQ(str, std::string("efgh567"));
  EXPECT_EQ(serial_port_->read(str, 2), 1);  // timeout
  EXPECT_EQ(str, std::string("efgh5678"));

  uint8_t cmd_vec[6] {0x12, 0x34, 0xF0, 0x0F, 0x00, 0xFF};
  std::vector<uint8_t> std_vec;
  ::write(master_fd_, cmd_vec, 6);
  EXPECT_EQ(serial_port_->read(std_vec, 3), 3);
  EXPECT_EQ(std_vec.size(), 3);
  EXPECT_EQ(serial_port_->read(std_vec, 0), 0);
  EXPECT_EQ(std_vec.size(), 3);
  EXPECT_EQ(serial_port_->read(std_vec, 2), 2);
  EXPECT_EQ(std_vec.size(), 5);
  EXPECT_EQ(serial_port_->read(std_vec, 2), 1);  // timeout
  EXPECT_EQ(std_vec.size(), 6);
  EXPECT_EQ(std_vec.front(), 0x12);
  EXPECT_EQ(std_vec.back(), 0xFF);

  uint8_t vec[6] {0x11, 0x11, 0x11, 0x11, 0x11, 0x11};
  ::write(master_fd_, cmd_vec, 6);
  EXPECT_EQ(serial_port_->read(vec, 3), 3);
  EXPECT_EQ(vec[0], 0x12);
  EXPECT_EQ(vec[1], 0x34);
  EXPECT_EQ(vec[2], 0xF0);
  EXPECT_EQ(serial_port_->read(vec, 0), 0);
  EXPECT_EQ(vec[3], 0x11);
  EXPECT_EQ(serial_port_->read(vec+3, 2), 2);
  EXPECT_EQ(vec[3], 0x0F);
  EXPECT_EQ(vec[4], 0x00);
  EXPECT_EQ(serial_port_->read(vec+5, 2), 1);  // timeout
  EXPECT_EQ(vec[5], 0xFF);
}

TEST_F(SerialTests, Writes) {
  char buf[5] = "";
  EXPECT_EQ(serial_port_->write(std::string("abcd")), 4);
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(std::string(buf, 4), std::string("abcd"));

  EXPECT_EQ(serial_port_->write(std::string("")), 0);

  std::vector<uint8_t> std_vec {0x12, 0x34, 0xFF, 0x00};
  EXPECT_EQ(serial_port_->write(std_vec), 4);
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(static_cast<uint8_t>(buf[0]), 0x12);
  EXPECT_EQ(static_cast<uint8_t>(buf[1]), 0x34);
  EXPECT_EQ(static_cast<uint8_t>(buf[2]), 0xFF);
  EXPECT_EQ(static_cast<uint8_t>(buf[3]), 0x00);

  uint8_t vec[4] {0x21, 0x43, 0xF0, 0x0F};
  EXPECT_EQ(serial_port_->write(vec, 4), 4);
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(static_cast<uint8_t>(buf[0]), 0x21);
  EXPECT_EQ(static_cast<uint8_t>(buf[1]), 0x43);
  EXPECT_EQ(static_cast<uint8_t>(buf[2]), 0xF0);
  EXPECT_EQ(static_cast<uint8_t>(buf[3]), 0x0F);
}

TEST_F(SerialTests, Timeout) {
  EXPECT_EQ(serial_port_->read(1), std::string(""));  // timeout
  ::write(master_fd_, "abcd", 4);
  EXPECT_EQ(serial_port_->read(4), std::string("abcd"));  // still works after a timeout
}

TEST_F(SerialTests, Flush) {
  ::write(master_fd_, "abcd1234", 8);
  EXPECT_NO_THROW(serial_port_->flush());
  EXPECT_EQ(serial_port_->read(1), std::string(""));  // timeout

  ::write(master_fd_, "abcd1234", 8);
  EXPECT_NO_THROW(serial_port_->flushInput());
  EXPECT_EQ(serial_port_->read(1), std::string(""));  // timeout

  char buf[5] = "";
  EXPECT_EQ(serial_port_->write("abcd"), 4);
  EXPECT_NO_THROW(serial_port_->flush());
  EXPECT_EQ(serial_port_->write("1234"), 4);
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(std::string(buf, 4), std::string("1234"));

  EXPECT_EQ(serial_port_->write("efgh"), 4);
  EXPECT_NO_THROW(serial_port_->flushOutput());
  EXPECT_EQ(serial_port_->write("5678"), 4);
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(std::string(buf, 4), std::string("5678"));
}
#endif

TEST(DummyTests, Dummy) {
  ASSERT_TRUE(true);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
