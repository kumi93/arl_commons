cmake_minimum_required(VERSION 2.8.3)
project(arl_commons)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        arl_hw_msgs
        std_msgs
        std_srvs
)

find_package(Boost)

catkin_package(
#        INCLUDE_DIRS include
#        CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
#add_executable(muscle_command_spamming_test_node src/muscle_command_spamming_test_node.cpp)
#add_dependencies(muscle_command_spamming_test_node arl_hw_msgs_gencpp)
#target_link_libraries(muscle_command_spamming_test_node ${catkin_LIBRARIES})

add_executable(rosjava_wrapper_node src/rosjava_wrapper_node.cpp)
add_dependencies(rosjava_wrapper_node arl_hw_msgs_gencpp)
target_link_libraries(rosjava_wrapper_node ${catkin_LIBRARIES})
