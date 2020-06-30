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
  ASSERT_TRUE(serial_port_->isOpen());
  ASSERT_THROW(serial_port_->open(), SerialException);  // already open
  ASSERT_TRUE(serial_port_->isOpen());
  serial_port_->close();
  ASSERT_FALSE(serial_port_->isOpen());
  ASSERT_NO_THROW(serial_port_->open());
  ASSERT_TRUE(serial_port_->isOpen());
}

TEST_F(SerialTests, Reads) {
  ::write(master_fd_, "abcd1234", 8);
  EXPECT_EQ(serial_port_->read(4), std::string("abcd"));
  EXPECT_EQ(serial_port_->read(3), std::string("123"));
  EXPECT_EQ(serial_port_->read(2), std::string("4"));  // timeout
  //TODO: add the other read methods and also partial reads
}

TEST_F(SerialTests, Writes) {
  serial_port_->write("abc\n");
  char buf[5] = "";
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(std::string(buf, 4), std::string("abc\n"));
  //TODO: add the other write methods
}

TEST_F(SerialTests, Timeout) {
  // Timeout a read, returns an empty string
  std::string empty = serial_port_->read();
  EXPECT_EQ(empty, std::string(""));

  // Ensure that writing/reading still works after a timeout.
  ::write(master_fd_, "abc\n", 4);
  std::string r = serial_port_->read(4);
  EXPECT_EQ(r, std::string("abc\n"));
}

TEST_F(SerialTests, Flush) {
  ::write(master_fd_, "abcd1234", 8);
  serial_port_->flush();
  EXPECT_EQ(serial_port_->read(1), std::string(""));  // timeout

  ::write(master_fd_, "abcd1234", 8);
  serial_port_->flushInput();
  EXPECT_EQ(serial_port_->read(1), std::string(""));  // timeout

  char buf[5] = "";
  serial_port_->write("abcd");
  serial_port_->flush();
  serial_port_->write("1234");
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(std::string(buf, 4), std::string("1234"));

  serial_port_->write("efgh");
  serial_port_->flushOutput();
  serial_port_->write("5678");
  ::read(master_fd_, buf, 4);
  EXPECT_EQ(std::string(buf, 4), std::string("5678"));
}

TEST_F(SerialTests, PartialRead) {
  // Write some data, but request more than was written.
  ::write(master_fd_, "abc\n", 4);

  // Should timeout, but return what was in the buffer.
  std::string empty = serial_port_->read(10);
  EXPECT_EQ(empty, std::string("abc\n"));

  // Ensure that writing/reading still works after a timeout.
  ::write(master_fd_, "abc\n", 4);
  std::string r = serial_port_->read(4);
  EXPECT_EQ(r, std::string("abc\n"));
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
