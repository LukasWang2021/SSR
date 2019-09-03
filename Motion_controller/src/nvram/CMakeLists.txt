cmake_minimum_required(VERSION 2.8.3)
project(nvram)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

if(catkin_CROSS)
    set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_CROSS})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_CROSS})
    set(CMAKE_CXX_FLAGS "-DCROSS_PLATFORM")
    set(CMAKE_C_FLAGS "-DCROSS_PLATFORM")
    message(STATUS "ARM-Cross")
else()
    set(catkin_INCLUDE_DIRS ${catkin_INCLUDE_DIRS_x86})
    set(catkin_LIBRARIES ${catkin_LIBRARYS_x86})
    message(STATUS "x86-64")
endif()

set(catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${depend_PATH_INC}/fst_spi/include
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES nvram
#  CATKIN_DEPENDS other_catkin_pkg
   DEPENDS fst_spi
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -g -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O0 -pthread -DPB_FIELD_32BIT -DSIMMULATION -DPRINT")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(nvram
   src/nvram.cpp
)


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(nvram
    ${catkin_LIBRARIES}
    fst_spi
)


install(TARGETS nvram
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
