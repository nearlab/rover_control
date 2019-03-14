cmake_minimum_required (VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/..)


project (rover_control)
SET(GCC_FLAGS "-g")
add_definitions(${GCC_FLAGS})
find_package(Eigen3 REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  kinova_driver
  vicon
)
catkin_package()
include_directories(
${catkin_INCLUDE_DIRS}
)
add_executable(rover_control src/rover_control.cpp)
target_link_libraries (rover_control serial ${catkin_LIBRARIES})
