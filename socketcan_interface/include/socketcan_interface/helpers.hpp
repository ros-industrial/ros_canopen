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

#ifndef SOCKETCAN_INTERFACE__HELPERS_HPP_
#define SOCKETCAN_INTERFACE__HELPERS_HPP_

#include <functional>

namespace can
{

template <typename T> class DelegateHelper : public T
{
  public:
  template <typename Object, typename Instance, typename ...Args>
  DelegateHelper(Object &&o, typename T::result_type (Instance::*member)(Args... args)) :
      T(
        [&o, fn=std::mem_fn(member)](Args... args) ->
        typename T::result_type
        {
          return fn(o, args...);
        })
  {
  }

  template <typename Callable>
  DelegateHelper(Callable &&c) : T(c)
  {}
};

}  // namespace can

#endif  // SOCKETCAN_INTERFACE__HELPERS_HPP_
