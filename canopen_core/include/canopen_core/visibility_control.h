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
#ifndef CANOPEN_CORE__VISIBILITY_CONTROL_H_
#define CANOPEN_CORE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CANOPEN_CORE_EXPORT __attribute__((dllexport))
#define CANOPEN_CORE_IMPORT __attribute__((dllimport))
#else
#define CANOPEN_CORE_EXPORT __declspec(dllexport)
#define CANOPEN_CORE_IMPORT __declspec(dllimport)
#endif
#ifdef CANOPEN_CORE_BUILDING_LIBRARY
#define CANOPEN_CORE_PUBLIC CANOPEN_CORE_EXPORT
#else
#define CANOPEN_CORE_PUBLIC CANOPEN_CORE_IMPORT
#endif
#define CANOPEN_CORE_PUBLIC_TYPE CANOPEN_CORE_PUBLIC
#define CANOPEN_CORE_LOCAL
#else
#define CANOPEN_CORE_EXPORT __attribute__((visibility("default")))
#define CANOPEN_CORE_IMPORT
#if __GNUC__ >= 4
#define CANOPEN_CORE_PUBLIC __attribute__((visibility("default")))
#define CANOPEN_CORE_LOCAL __attribute__((visibility("hidden")))
#else
#define CANOPEN_CORE_PUBLIC
#define CANOPEN_CORE_LOCAL
#endif
#define CANOPEN_CORE_PUBLIC_TYPE
#endif

#endif  // CANOPEN_CORE__VISIBILITY_CONTROL_H_
