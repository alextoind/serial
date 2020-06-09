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

#include "gtest/gtest.h"
#include "serial/impl/impl.h"

#if defined(__linux__)
#include <unistd.h>
#include <stdlib.h>

using serial::MillisecondTimer;
#endif

namespace {

#if defined(__linux__)
/**
 * Do 100 trials of timing gaps between 0 and 19 milliseconds.
 * Expect accuracy within one millisecond.
 */
TEST(timer_tests, short_intervals) {
  for (int trial = 0; trial < 100; trial++)
  {
    uint32_t ms = rand() % 20;
    MillisecondTimer mt(ms);
    usleep(1000 * ms);
    int32_t r = mt.remaining(); 

    // 1ms slush, for the cost of calling usleep.
    EXPECT_NEAR(r+1, 0, 1);
  }
}

TEST(timer_tests, overlapping_long_intervals) {
  MillisecondTimer* timers[10];

  // Experimentally determined. Corresponds to the extra time taken by the loops,
  // the big usleep, and the test infrastructure itself.
  const int slush_factor = 14;

  // Set up the timers to each time one second, 1ms apart.
  for (int t = 0; t < 10; t++)
  {
    timers[t] = new MillisecondTimer(1000);
    usleep(1000);
  }

  // Check in on them after 500ms.
  usleep(500000);
  for (int t = 0; t < 10; t++)
  {
    EXPECT_NEAR(timers[t]->remaining(), 500 - slush_factor + t, 5);
  }

  // Check in on them again after another 500ms and free them.
  usleep(500000);
  for (int t = 0; t < 10; t++)
  {
    EXPECT_NEAR(timers[t]->remaining(), -slush_factor + t, 5);
    delete timers[t];
  }
}
#endif

TEST(timer_tests, dummy) {
  ASSERT_TRUE(true);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
