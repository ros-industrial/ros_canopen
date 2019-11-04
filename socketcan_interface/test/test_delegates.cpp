// Copyright (c) 2016-2019, Fraunhofer, Mathias Lüdtke, AutonomouStuff
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

#include <socketcan_interface/delegates.hpp>
#include <socketcan_interface/dummy.hpp>
#include <boost/bind.hpp>

#include <list>
#include <memory>
#include <string>

class Receiver
{
public:
  std::list<std::string> responses;
  void handle(const can::Frame & f)
  {
    responses.push_back(can::tostring(f, true));
  }

  Receiver() = default;
  ~Receiver() = default;

private:
  Receiver(const Receiver &) = delete;
  Receiver & operator=(const Receiver &) = delete;
};

Receiver g_r2;

void fill_r2(const can::Frame & f)
{
  g_r2.handle(f);
}

void fill_any(Receiver & r, const can::Frame & f)
{
  r.handle(f);
}

TEST(DelegatesTest, testFrameDelegate)
{
  can::DummyInterface dummy(true);
  Receiver r1, r3, r4, r5;
  boost::shared_ptr<Receiver> r6(new Receiver());
  std::shared_ptr<Receiver> r7(new Receiver());

  can::FrameListenerConstSharedPtr l1 =
    dummy.createMsgListener(can::CommInterface::FrameDelegate(&r1, &Receiver::handle));
  can::FrameListenerConstSharedPtr l2 =
    dummy.createMsgListener(can::CommInterface::FrameDelegate(fill_r2));
  can::FrameListenerConstSharedPtr l3 =
    dummy.createMsgListener(
    can::CommInterface::FrameDelegate(
      std::bind(fill_any, std::ref(r3), std::placeholders::_1)));
  can::FrameListenerConstSharedPtr l4 =
    dummy.createMsgListener(std::bind(fill_any, std::ref(r4), std::placeholders::_1));
  can::FrameListenerConstSharedPtr l5 =
    dummy.createMsgListener(boost::bind(fill_any, boost::ref(r5), boost::placeholders::_1));
  can::FrameListenerConstSharedPtr l6 =
    dummy.createMsgListener(can::CommInterface::FrameDelegate(r6, &Receiver::handle));
  can::FrameListenerConstSharedPtr l7 =
    dummy.createMsgListener(can::CommInterface::FrameDelegate(r7, &Receiver::handle));

  std::list<std::string> expected;
  dummy.send(can::toframe("0#8200"));
  expected.push_back("0#8200");

  EXPECT_EQ(expected, r1.responses);
  EXPECT_EQ(expected, g_r2.responses);
  EXPECT_EQ(expected, r3.responses);
  EXPECT_EQ(expected, r4.responses);
  EXPECT_EQ(expected, r5.responses);
  EXPECT_EQ(expected, r6->responses);
  EXPECT_EQ(expected, r7->responses);
}

class BoolTest
{
  bool ret_;

public:
  bool test_bool()
  {
    return ret_;
  }

  explicit BoolTest(bool ret)
  : ret_(ret)
  {}

  BoolTest(const BoolTest &) = delete;
  BoolTest & operator=(const BoolTest &) = delete;
};

TEST(DelegatesTest, testBoolFunc)
{
  using BoolFunc = std::function<bool (void)>;
  using BoolDelegate = can::DelegateHelper<BoolFunc>;

  BoolDelegate d1([]() {return false;});
  BoolDelegate d2([]() {return true;});

  EXPECT_FALSE(d1());
  EXPECT_TRUE(d2());

  BoolTest b1(false);
  BoolTest b2(true);

  BoolDelegate d3(&b1, &BoolTest::test_bool);
  BoolDelegate d4(&b2, &BoolTest::test_bool);
  EXPECT_FALSE(d3());
  EXPECT_TRUE(d4());
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
