// Copyright (c) 2016-2019, Mathias Lüdtke, AutonomouStuff
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <gtest/gtest.h>

#include <string>

#include "socketcan_interface/filter.hpp"
#include "socketcan_interface/string.hpp"
#include "socketcan_interface/dummy.hpp"

TEST(FilterTest, simpleMask)
{
  const std::string msg1("123#");
  const std::string msg2("124#");

  can::FrameFilterSharedPtr f1 = can::tofilter("123");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
}

TEST(FilterTest, maskTests)
{
  const std::string msg1("123#");
  const std::string msg2("124#");
  const std::string msg3("122#");

  can::FrameFilterSharedPtr f1 = can::tofilter("123:123");
  can::FrameFilterSharedPtr f2 = can::tofilter("123:ffe");
  can::FrameFilterSharedPtr f3 = can::tofilter("123~123");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
  EXPECT_FALSE(f1->pass(can::toframe(msg3)));


  EXPECT_TRUE(f2->pass(can::toframe(msg1)));
  EXPECT_FALSE(f2->pass(can::toframe(msg2)));
  EXPECT_TRUE(f2->pass(can::toframe(msg3)));

  EXPECT_FALSE(f3->pass(can::toframe(msg1)));
  EXPECT_TRUE(f3->pass(can::toframe(msg2)));
  EXPECT_TRUE(f3->pass(can::toframe(msg3)));
}

TEST(FilterTest, rangeTest)
{
  const std::string msg1("120#");
  const std::string msg2("125#");
  const std::string msg3("130#");

  can::FrameFilterSharedPtr f1 = can::tofilter("120-120");
  can::FrameFilterSharedPtr f2 = can::tofilter("120_120");
  can::FrameFilterSharedPtr f3 = can::tofilter("120-125");

  EXPECT_TRUE(f1->pass(can::toframe(msg1)));
  EXPECT_FALSE(f1->pass(can::toframe(msg2)));
  EXPECT_FALSE(f1->pass(can::toframe(msg3)));

  EXPECT_FALSE(f2->pass(can::toframe(msg1)));
  EXPECT_TRUE(f2->pass(can::toframe(msg2)));
  EXPECT_TRUE(f2->pass(can::toframe(msg3)));

  EXPECT_TRUE(f3->pass(can::toframe(msg1)));
  EXPECT_TRUE(f3->pass(can::toframe(msg2)));
  EXPECT_FALSE(f3->pass(can::toframe(msg3)));
}

class Counter
{
public:
  size_t count_;
  Counter()
  : count_(0) {}

  void count(const can::Frame & frame)
  {
    ++count_;
  }
};

TEST(FilterTest, listenerTest)
{
  Counter counter;
  can::CommInterfaceSharedPtr dummy(new can::DummyInterface(true));

  can::FilteredFrameListener::FilterVector filters;
  filters.push_back(can::tofilter("123:FFE"));

  can::FrameListenerConstSharedPtr listener(
<<<<<<< HEAD
    new can::FilteredFrameListener(
      dummy, std::bind(&Counter::count, std::ref(counter), std::placeholders::_1), filters));
=======
      new can::FilteredFrameListener(
        dummy,
        can::CommInterface::FrameDelegate(
          can::CommInterface::createFrameDelegate(&counter, &Counter::count)),
        filters));
>>>>>>> Replacing FastDelegate with std::function and std::bind.

  can::Frame f1 = can::toframe("123#");
  can::Frame f2 = can::toframe("124#");
  can::Frame f3 = can::toframe("122#");

  dummy->send(f1);
  EXPECT_EQ(1, counter.count_);
  dummy->send(f2);
  EXPECT_EQ(1, counter.count_);
  dummy->send(f3);
  EXPECT_EQ(2, counter.count_);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
