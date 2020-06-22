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
#include <serial/serial.h>

using namespace serial;

namespace {

#if defined(__linux__)
class TimeoutTests : public ::testing::Test {
 protected:
  void SetUp() override {
    simple_timeout = std::unique_ptr<Serial::Timeout>(new Serial::Timeout(25));
    complete_timeout = std::unique_ptr<Serial::Timeout>(new Serial::Timeout(0, 10, 20, 100, 0));
  }

  static std::chrono::milliseconds cycleRead(const std::unique_ptr<Serial::Timeout> &timeout, const int &cycles, const size_t &multiplier) {
    std::chrono::milliseconds remaining_time(0);
    for (int i=0; i<cycles; i++) {
      auto read_deadline = timeout->getReadDeadline(multiplier);
      usleep(std::chrono::duration_cast<std::chrono::microseconds>(timeout->getReadConstant() + timeout->getReadMultiplier() * multiplier).count());
      remaining_time += Serial::Timeout::remainingMilliseconds(read_deadline);
    }
    return remaining_time;
  }

  static std::chrono::milliseconds cycleWrite(const std::unique_ptr<Serial::Timeout> &timeout, const int &cycles, const size_t &multiplier) {
    std::chrono::milliseconds remaining_time(0);
    for (int i=0; i<cycles; i++) {
      auto write_deadline = timeout->getWriteDeadline(multiplier);
      usleep(std::chrono::duration_cast<std::chrono::microseconds>(timeout->getWriteConstant() + timeout->getWriteMultiplier() * multiplier).count());
      remaining_time += Serial::Timeout::remainingMilliseconds(write_deadline);
    }
    return remaining_time;
  }

  std::unique_ptr<Serial::Timeout> simple_timeout;
  std::unique_ptr<Serial::Timeout> complete_timeout;
};

TEST_F(TimeoutTests, SimpleTimeoutRead) {
  EXPECT_NEAR(cycleRead(simple_timeout, 8, 0).count(), 0, 2);
  EXPECT_NEAR(cycleRead(simple_timeout, 8, 10).count(), 0, 2);
}

TEST_F(TimeoutTests, SimpleTimeoutWrite) {
  EXPECT_NEAR(cycleWrite(simple_timeout, 8, 0).count(), 0, 2);
  EXPECT_NEAR(cycleWrite(simple_timeout, 8, 10).count(), 0, 2);
}

TEST_F(TimeoutTests, CompleteTimeoutRead) {
  EXPECT_NEAR(cycleRead(complete_timeout, 20, 0).count(), 0, 2);
  EXPECT_NEAR(cycleRead(complete_timeout, 4, 2).count(), 0, 2);
}

TEST_F(TimeoutTests, CompleteTimeoutWrite) {
  EXPECT_NEAR(cycleWrite(complete_timeout, 2, 0).count(), 0, 2);
  EXPECT_NEAR(cycleWrite(complete_timeout, 2, 1000).count(), 0, 2);
}
#endif

TEST(DummyTests, Dummy) {
  ASSERT_TRUE(true);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
