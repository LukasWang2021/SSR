cmake_minimum_required(VERSION 2.8.3)
project(nvram_test)

SET(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/nvram_test)

include_directories(
  include
  ../../yaml_help/include
  ../../log_manager/include
  ../../thread_help/include 
  ../../rtm_spi/include
  ../../nvram/include
  ${rtm_INCLUDE_DIRS}
)

add_executable(nvram_test
    nvram_test.cpp
)

target_link_libraries(nvram_test
    ${rtm_LIBRARIES}
    nvram_handler
)

add_executable(read_nvram
    read_nvram.cpp
)

target_link_libraries(read_nvram
    ${rtm_LIBRARIES}
    nvram_handler
    rtm_spi
)


add_executable(write_nvram
    write_nvram.cpp
)

target_link_libraries(write_nvram
    ${rtm_LIBRARIES}
    nvram_handler
    rtm_spi
)

