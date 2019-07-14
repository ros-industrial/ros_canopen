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
