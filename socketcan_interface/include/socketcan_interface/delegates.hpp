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

#ifndef SOCKETCAN_INTERFACE_DELEGATES_H_
#define SOCKETCAN_INTERFACE_DELEGATES_H_

#include <functional>

namespace can
{

template <typename T> class DelegateHelper : public T {
public:
  template <typename Object, typename Instance, typename ...Args>
  DelegateHelper(Object &&o, typename T::result_type (Instance::*member)(Args... args)) :
      T([o, member](Args... args) -> typename T::result_type { return ((*o).*member)(args...); })
  {
  }
  template <typename Callable>
  DelegateHelper(Callable &&c) : T(c)
  {
  }
};

}  // namespace can

#endif  // SOCKETCAN_INTERFACE_DELEGATES_H_
