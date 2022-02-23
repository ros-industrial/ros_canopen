#ifndef PROXY_DEVICE_DRIVER__VISIBILITY_CONTROL_H_
#define PROXY_DEVICE_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PROXY_DEVICE_DRIVER_EXPORT __attribute__ ((dllexport))
    #define PROXY_DEVICE_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define PROXY_DEVICE_DRIVER_EXPORT __declspec(dllexport)
    #define PROXY_DEVICE_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef PROXY_DEVICE_DRIVER_BUILDING_LIBRARY
    #define PROXY_DEVICE_DRIVER_PUBLIC PROXY_DEVICE_DRIVER_EXPORT
  #else
    #define PROXY_DEVICE_DRIVER_PUBLIC PROXY_DEVICE_DRIVER_IMPORT
  #endif
  #define PROXY_DEVICE_DRIVER_PUBLIC_TYPE PROXY_DEVICE_DRIVER_PUBLIC
  #define PROXY_DEVICE_DRIVER_LOCAL
#else
  #define PROXY_DEVICE_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define PROXY_DEVICE_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define PROXY_DEVICE_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define PROXY_DEVICE_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PROXY_DEVICE_DRIVER_PUBLIC
    #define PROXY_DEVICE_DRIVER_LOCAL
  #endif
  #define PROXY_DEVICE_DRIVER_PUBLIC_TYPE
#endif

#endif  // PROXY_DEVICE_DRIVER__VISIBILITY_CONTROL_H_
