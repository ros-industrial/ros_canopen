#ifndef MOTION_CONTROLLER_DRIVER__VISIBILITY_CONTROL_H_
#define MOTION_CONTROLLER_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MOTION_CONTROLLER_DRIVER_EXPORT __attribute__ ((dllexport))
    #define MOTION_CONTROLLER_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define MOTION_CONTROLLER_DRIVER_EXPORT __declspec(dllexport)
    #define MOTION_CONTROLLER_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef MOTION_CONTROLLER_DRIVER_BUILDING_LIBRARY
    #define MOTION_CONTROLLER_DRIVER_PUBLIC MOTION_CONTROLLER_DRIVER_EXPORT
  #else
    #define MOTION_CONTROLLER_DRIVER_PUBLIC MOTION_CONTROLLER_DRIVER_IMPORT
  #endif
  #define MOTION_CONTROLLER_DRIVER_PUBLIC_TYPE MOTION_CONTROLLER_DRIVER_PUBLIC
  #define MOTION_CONTROLLER_DRIVER_LOCAL
#else
  #define MOTION_CONTROLLER_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define MOTION_CONTROLLER_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define MOTION_CONTROLLER_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define MOTION_CONTROLLER_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MOTION_CONTROLLER_DRIVER_PUBLIC
    #define MOTION_CONTROLLER_DRIVER_LOCAL
  #endif
  #define MOTION_CONTROLLER_DRIVER_PUBLIC_TYPE
#endif

#endif  // MOTION_CONTROLLER_DRIVER__VISIBILITY_CONTROL_H_
