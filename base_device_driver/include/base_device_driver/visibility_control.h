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

#ifndef BASE_DEVICE_DRIVER__VISIBILITY_CONTROL_H_
#define BASE_DEVICE_DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BASE_DEVICE_DRIVER_EXPORT __attribute__ ((dllexport))
    #define BASE_DEVICE_DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define BASE_DEVICE_DRIVER_EXPORT __declspec(dllexport)
    #define BASE_DEVICE_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef BASE_DEVICE_DRIVER_BUILDING_LIBRARY
    #define BASE_DEVICE_DRIVER_PUBLIC BASE_DEVICE_DRIVER_EXPORT
  #else
    #define BASE_DEVICE_DRIVER_PUBLIC BASE_DEVICE_DRIVER_IMPORT
  #endif
  #define BASE_DEVICE_DRIVER_PUBLIC_TYPE BASE_DEVICE_DRIVER_PUBLIC
  #define BASE_DEVICE_DRIVER_LOCAL
#else
  #define BASE_DEVICE_DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define BASE_DEVICE_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define BASE_DEVICE_DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define BASE_DEVICE_DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BASE_DEVICE_DRIVER_PUBLIC
    #define BASE_DEVICE_DRIVER_LOCAL
  #endif
  #define BASE_DEVICE_DRIVER_PUBLIC_TYPE
#endif

#endif  // BASE_DEVICE_DRIVER__VISIBILITY_CONTROL_H_
