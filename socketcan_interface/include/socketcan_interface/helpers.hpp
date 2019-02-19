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
