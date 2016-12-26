cmake_minimum_required(VERSION 2.8.3)
project(arl_commons)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        arl_hw_msgs
        std_msgs
        )

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
)

## Declare a C++ executable
add_executable(muscle_command_spamming_test_node src/muscle_command_spamming_test_node.cpp)
add_dependencies(muscle_command_spamming_test_node arl_hw_msgs_gencpp)
target_link_libraries(muscle_command_spamming_test_node ${catkin_LIBRARIES})
