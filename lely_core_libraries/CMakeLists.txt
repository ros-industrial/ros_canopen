cmake_minimum_required(VERSION 3.8)
project(lely_core_libraries)
find_package(ament_cmake REQUIRED)

include(ExternalProject)
ExternalProject_Add(upstr_lely_core_libraries    # Name for custom target
  #--Download step--------------
  GIT_REPOSITORY https://gitlab.com/lely_industries/lely-core.git
  GIT_TAG ac31cd5cdf2ded4b2a5ba4a94520f388450614ff
  TIMEOUT 60          
  #--Update/Patch step----------
  UPDATE_COMMAND touch <SOURCE_DIR>/config.h 
    COMMAND mkdir -p <SOURCE_DIR>/include <SOURCE_DIR>/lib 
    COMMAND touch <SOURCE_DIR>/python/dcf-tools/README.md
  #--Configure step-------------
  CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR> COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-cython --disable-doc --disable-tests --disable-static "CPPFLAGS=-DNDEBUG" "CFLAGS=-DNDEBUG" "CXXFLAGS=-DNDEBUG"
  #--Build step-----------------
  BUILD_IN_SOURCE 1         # Use source dir for build dir
  #--Install step---------------
  INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}"  # Installation prefix
  BUILD_COMMAND cd <SOURCE_DIR>  COMMAND $(MAKE) --silent
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin/ DESTINATION bin
        PATTERN "bin/*"
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        GROUP_EXECUTE GROUP_READ)

set(lely_core_cmake_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("cmake/lely_core_libraries-extras.cmake" NO_POLICY_SCOPE)


install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(lely-can lely-co lely-coapp lely-ev lely-io2 lely-io lely-libc lely-tap lely-util)
ament_package(
  CONFIG_EXTRAS
  "cmake/lely_core_libraries-extras.cmake"
)
