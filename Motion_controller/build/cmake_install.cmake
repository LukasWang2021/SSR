# Install script for directory: /home/ld/git_work/mc_application/Motion_controller/src

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
  include("/home/ld/git_work/mc_application/Motion_controller/build/pb/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/protos/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/xml_help/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/yaml_help/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/sem_help/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/thread_help/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/basic_alg/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/log_manager/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/error_queue/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/tp_comm/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/file_manager/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/system_model_manager/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/algorithm_base/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/axis/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/axis1000/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/group/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/core_comm/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/servo_comm/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/test/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/base_device/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/io_1000/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/io_safety/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/rtm_spi/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/nvram/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/reg_manager/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/joint_constraint/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/kinematics_alg/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/dynamic_alg/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/transformation/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/coordinate_manager/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/tool_manager/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/trajectory_planner/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/sensor_process/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/motion_control/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/controller/cmake_install.cmake")
  include("/home/ld/git_work/mc_application/Motion_controller/build/embed_python/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ld/git_work/mc_application/Motion_controller/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
