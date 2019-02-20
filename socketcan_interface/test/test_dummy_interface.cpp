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
#include <list>

#include "socketcan_interface/dispatcher.hpp"
#include "socketcan_interface/dummy.hpp"

class DummyInterfaceTest
: public ::testing::Test
{
public:
  std::list<std::string> responses;
  can::DummyInterface dummy;
  DummyInterfaceTest()
  : dummy(true), listener(dummy.createMsgListenerM(this, &DummyInterfaceTest::handle)) { }

  void handle(const can::Frame & f)
  {
    responses.push_back(can::tostring(f, true));
  }

  can::FrameListenerConstSharedPtr listener;
};

// Declare a test
TEST_F(DummyInterfaceTest, testCase1)
{
  dummy.add("0#8200", "701#00", false);

  std::list<std::string> expected;

  dummy.send(can::toframe("0#8200"));
  expected.push_back("0#8200");
  expected.push_back("701#00");

  EXPECT_EQ(expected, responses);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
