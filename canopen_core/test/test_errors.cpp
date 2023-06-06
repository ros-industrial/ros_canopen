//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#include <chrono>
#include "canopen_core/device_container_error.hpp"
#include "canopen_core/driver_error.hpp"
#include "canopen_core/master_error.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace testing;

TEST(DriverErrorTest, test_what)
{
  DriverException test_obj("hello");
  EXPECT_TRUE(std::string("hello").compare(test_obj.what()) == 0);
}

TEST(MasterErrorTest, test_what)
{
  MasterException test_obj("hello");
  EXPECT_TRUE(std::string("hello").compare(test_obj.what()) == 0);
}

TEST(DeviceContainerErrorTest, test_what)
{
  DeviceContainerException test_obj("hello");
  EXPECT_TRUE(std::string("hello").compare(test_obj.what()) == 0);
}
