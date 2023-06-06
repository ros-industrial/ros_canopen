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

#ifndef DRIVER_ERROR_HPP_
#define DRIVER_ERROR_HPP_

#include <string>
#include <system_error>

namespace ros2_canopen
{
/**
 * @brief Driver Exception
 *
 * This exception is used, when a driver
 * fails.
 *
 */
class DriverException : public std::exception
{
private:
  std::string what_;

public:
  DriverException(std::string what) { what_ = what; }

  char * what();
};

}  // namespace ros2_canopen

#endif
