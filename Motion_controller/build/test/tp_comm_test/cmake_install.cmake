# Install script for directory: /home/ld/git_work/mc_application/Motion_controller/src/test/tp_comm_test

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_axis/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_common/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_controller/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_event/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_io/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_servo_sampling/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_servo1001/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_group/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_frame/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_motion_control/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_reg/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/tp_comm_test/src/test_interpreter/cmake_install.cmake")

endif()

