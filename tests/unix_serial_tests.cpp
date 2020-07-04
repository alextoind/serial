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
    EXPECT_NO_THROW(serial_port_ = std::unique_ptr<Serial>(new Serial(std::string(pty_name), 9600, Serial::Timeout(100))));
    EXPECT_EQ(serial_port_->getPort(), std::string(pty_name));
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
  EXPECT_TRUE(serial_port_->isOpen());
  EXPECT_THROW(serial_port_->open(), SerialException);  // already open
  EXPECT_TRUE(serial_port_->isOpen());
  EXPECT_NO_THROW(serial_port_->close());
  EXPECT_FALSE(serial_port_->isOpen());
  EXPECT_NO_THROW(serial_port_->open());
  EXPECT_TRUE(serial_port_->isOpen());

  std::string port = serial_port_->getPort();
  EXPECT_NO_THROW(serial_port_->setPort(port));
  EXPECT_TRUE(serial_port_->isOpen());
  EXPECT_THROW(serial_port_->setPort(""), std::invalid_argument);
  EXPECT_FALSE(serial_port_->isOpen());
  EXPECT_NO_THROW(serial_port_->setPort("/dev/no_real_port"));
  EXPECT_THROW(serial_port_->open(), IOException);  // invalid serial port
  EXPECT_FALSE(serial_port_->isOpen());
  EXPECT_NO_THROW(serial_port_->setPort(port));
  EXPECT_FALSE(serial_port_->isOpen());
  EXPECT_NO_THROW(serial_port_->open());
  EXPECT_TRUE(serial_port_->isOpen());
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

TEST_F(SerialTests, ReadLines) {
  ::write(master_fd_, "abcd1234\nefgh5678\nil\r9", 22);
  EXPECT_EQ(serial_port_->readline(), std::string("abcd1234\n"));
  EXPECT_EQ(serial_port_->readline(), std::string("efgh5678\n"));
  EXPECT_EQ(serial_port_->readline(), std::string("il\r9"));  // timeout

  std::string line;
  ::write(master_fd_, "abcd1234\nefgh5678\nil\r9", 22);
  EXPECT_EQ(serial_port_->readline(line), 9);
  EXPECT_EQ(line, std::string("abcd1234\n"));
  EXPECT_EQ(serial_port_->readline(line, 0), 0);
  EXPECT_EQ(line, std::string("abcd1234\n"));
  EXPECT_EQ(serial_port_->readline(line, 4), 4);
  EXPECT_EQ(line, std::string("abcd1234\nefgh"));
  EXPECT_EQ(serial_port_->readline(line, 10, "7"), 3);
  EXPECT_EQ(line, std::string("abcd1234\nefgh567"));
  EXPECT_EQ(serial_port_->readline(line, 10, ""), 1);
  EXPECT_EQ(line, std::string("abcd1234\nefgh5678"));
  EXPECT_EQ(serial_port_->readline(line, 10, "il\r"), 4);
  EXPECT_EQ(line, std::string("abcd1234\nefgh5678\nil\r"));
  EXPECT_EQ(serial_port_->readline(line, 10, "il\r"), 1);  // timeout
  EXPECT_EQ(line, std::string("abcd1234\nefgh5678\nil\r9"));

  std::vector<std::string> lines;
  ::write(master_fd_, "abcd1234\nefgh5678\nil\r9", 22);
  EXPECT_NO_THROW(lines = serial_port_->readlines());  // timeout
  EXPECT_EQ(lines.size(), 3);
  EXPECT_EQ(lines.at(0), std::string("abcd1234\n"));
  EXPECT_EQ(lines.at(1), std::string("efgh5678\n"));
  EXPECT_EQ(lines.at(2), std::string("il\r9"));

  ::write(master_fd_, "abcd1234\nefgh5678\nil\r9", 22);
  EXPECT_NO_THROW(lines = serial_port_->readlines(0));
  EXPECT_EQ(lines.size(), 0);
  EXPECT_NO_THROW(lines = serial_port_->readlines(8, ""));
  EXPECT_EQ(lines.size(), 8);
  EXPECT_EQ(lines.at(0), std::string("a"));
  EXPECT_EQ(lines.at(7), std::string("4"));
  EXPECT_NO_THROW(lines = serial_port_->readlines(22, "xyz"));  // timeout
  EXPECT_EQ(lines.size(), 1);
  EXPECT_EQ(lines.at(0), std::string("\nefgh5678\nil\r9"));
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

#if defined(__linux__)  // does not work on macOS yet
  Serial::Timeout simple_timeout(50);
  serial_port_->setTimeout(simple_timeout);
  EXPECT_EQ(serial_port_->getTimeout().getInterByteMilliseconds(), std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(serial_port_->getTimeout().getReadConstantMilliseconds(), 50);
  EXPECT_EQ(serial_port_->getTimeout().getReadMultiplierMilliseconds(), 0);
  EXPECT_EQ(serial_port_->getTimeout().getWriteConstantMilliseconds(), 50);
  EXPECT_EQ(serial_port_->getTimeout().getWriteMultiplierMilliseconds(), 0);
  auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(serial_port_->read(1), std::string(""));  // timeout
  EXPECT_NEAR(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count(), serial_port_->getTimeout().getReadConstant().count(), 5);
  start = std::chrono::steady_clock::now();
  EXPECT_EQ(serial_port_->read(5), std::string(""));  // timeout
  EXPECT_NEAR(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count(), serial_port_->getTimeout().getReadConstant().count(), 5);

  serial_port_->setTimeout(0, 100, 20, 100, 0);  // complete_timeout
  EXPECT_EQ(serial_port_->getTimeout().getInterByteMilliseconds(), 0);
  EXPECT_EQ(serial_port_->getTimeout().getReadConstantMilliseconds(), 100);
  EXPECT_EQ(serial_port_->getTimeout().getReadMultiplierMilliseconds(), 20);
  EXPECT_EQ(serial_port_->getTimeout().getWriteConstantMilliseconds(), 100);
  EXPECT_EQ(serial_port_->getTimeout().getWriteMultiplierMilliseconds(), 0);
  start = std::chrono::steady_clock::now();
  EXPECT_EQ(serial_port_->read(1), std::string(""));  // timeout
  EXPECT_NEAR(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count(), (serial_port_->getTimeout().getReadConstant() + 1*serial_port_->getTimeout().getReadMultiplier()).count(), 5);
  start = std::chrono::steady_clock::now();
  EXPECT_EQ(serial_port_->read(5), std::string(""));  // timeout
  EXPECT_NEAR(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count(), (serial_port_->getTimeout().getReadConstant() + 5*serial_port_->getTimeout().getReadMultiplier()).count(), 5);
#endif
}

TEST_F(SerialTests, AvailableAndWait) {
  EXPECT_EQ(serial_port_->available(), 0);
  ::write(master_fd_, "abcd", 4);
  EXPECT_TRUE(serial_port_->waitReadable());
  EXPECT_EQ(serial_port_->available(), 4);
  EXPECT_EQ(serial_port_->read(2), std::string("ab"));
  EXPECT_EQ(serial_port_->available(), 2);
  ::write(master_fd_, "efgh", 4);
  serial_port_->waitByteTimes(4);  // waitReadable() is already true (there are 2 bytes in the buffer)
  EXPECT_EQ(serial_port_->available(), 6);
  EXPECT_EQ(serial_port_->read(8), std::string("cdefgh"));  //timeout
  EXPECT_EQ(serial_port_->available(), 0);
  EXPECT_FALSE(serial_port_->waitReadable());  //timeout
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
