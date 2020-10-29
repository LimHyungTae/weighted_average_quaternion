cmake_minimum_required(VERSION 2.8.3)
project(wma)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    message_generation
#    pcl_conversions
#    pcl_ros
)

generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    sensor_msgs
   )

find_package(Boost REQUIRED)
find_package (Eigen3 REQUIRED)
MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}")

#find_package(PCL 1.7 REQUIRED)
#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()
#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES beginner_tutorials
    #  CATKIN_DEPENDS roscpp rospy std_msgs
    #  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

include_directories(${Boost_INCLUDE_DIRS})


SET(CMAKE_CXX_FLAGS "-std=c++17 -O2 -g -Wall -msse3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


add_executable(wma_theta src/wma_theta.cpp src/wma.hpp)
add_dependencies(wma_theta wma_generate_messages_cpp)
target_link_libraries(wma_theta ${catkin_LIBRARIES})

add_executable(wma_q src/wma_quaternions.cpp src/wma.hpp)
add_dependencies(wma_q wma_generate_messages_cpp)
target_link_libraries(wma_q ${catkin_LIBRARIES})
