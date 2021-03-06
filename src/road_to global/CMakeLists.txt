cmake_minimum_required(VERSION 3.0.2)
project(road_to_global)

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

add_executable(road_to_global src/road_to_global.cpp)
target_link_libraries(road_to_global ${catkin_LIBRARIES})
add_dependencies(road_to_global ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



