cmake_minimum_required(VERSION 3.0.2)
project(btraj)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
)

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable(
    btraj_main
    src/btraj_main.cpp
)
target_link_libraries(btraj_main ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)
