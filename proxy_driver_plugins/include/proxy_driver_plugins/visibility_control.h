#ifndef PROXY_DRIVER_PLUGINS__VISIBILITY_CONTROL_H_
#define PROXY_DRIVER_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PROXY_DRIVER_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define PROXY_DRIVER_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define PROXY_DRIVER_PLUGINS_EXPORT __declspec(dllexport)
    #define PROXY_DRIVER_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef PROXY_DRIVER_PLUGINS_BUILDING_LIBRARY
    #define PROXY_DRIVER_PLUGINS_PUBLIC PROXY_DRIVER_PLUGINS_EXPORT
  #else
    #define PROXY_DRIVER_PLUGINS_PUBLIC PROXY_DRIVER_PLUGINS_IMPORT
  #endif
  #define PROXY_DRIVER_PLUGINS_PUBLIC_TYPE PROXY_DRIVER_PLUGINS_PUBLIC
  #define PROXY_DRIVER_PLUGINS_LOCAL
#else
  #define PROXY_DRIVER_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define PROXY_DRIVER_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define PROXY_DRIVER_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define PROXY_DRIVER_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PROXY_DRIVER_PLUGINS_PUBLIC
    #define PROXY_DRIVER_PLUGINS_LOCAL
  #endif
  #define PROXY_DRIVER_PLUGINS_PUBLIC_TYPE
#endif

#endif  // PROXY_DRIVER_PLUGINS__VISIBILITY_CONTROL_H_
