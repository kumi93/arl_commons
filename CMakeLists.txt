cmake_minimum_required(VERSION 2.8.3)
project(arl_commons)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        arl_hw_msgs
        std_msgs
        std_srvs
        tf
        visualization_msgs
)

find_package(Boost)

catkin_package(
#        INCLUDE_DIRS include
#        CATKIN_DEPENDS roscpp
)
catkin_python_setup()

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)
