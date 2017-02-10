cmake_minimum_required(VERSION 2.8.0)
project(asctec_uav_msgs)

if(CATKIN_DEVEL_PREFIX)
  MESSAGE(STATUS "Building as catkin package.")
  find_package(catkin_simple REQUIRED)
  catkin_simple(ALL_DEPS_REQUIRED)
  cs_install()
  cs_export()
else(CATKIN_DEVEL_PREFIX)
  MESSAGE(STATUS "Building as native cmake package.")
  include_directories(include)
  # TODO: install, pkg-config?
endif(CATKIN_DEVEL_PREFIX)
