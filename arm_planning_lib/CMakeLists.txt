cmake_minimum_required(VERSION 2.8.3)
project(arm_planning_lib)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cwru_action
  baxter_traj_streamer
)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(arm_planning_lib src/arm_planning_lib.cpp) 

# Executables
cs_add_executable(arm_planning_lib_test src/arm_planning_lib_test.cpp)
target_link_libraries(arm_planning_lib_test arm_planning_lib baxter_traj_streamer)

cs_install()
cs_export()
    
