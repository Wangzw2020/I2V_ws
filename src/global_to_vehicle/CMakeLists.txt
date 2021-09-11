cmake_minimum_required(VERSION 3.0.2)
project(global_to_vehicle)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  perception_msgs
  #GPS
  serial
  roslib
  roscpp
  rosconsole
  tf
  nav_msgs
  sensor_msgs
)


find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Boost REQUIRED COMPONENTS 
	system 
	filesystem 
	thread
)

catkin_package(
)

set(EIGEN_INC_DIR /usr/include/eigen3/)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INC_DIR}
)

add_executable(global_to_vehicle src/global_to_vehicle.cpp)
target_link_libraries(global_to_vehicle ${catkin_LIBRARIES})
add_dependencies(global_to_vehicle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



