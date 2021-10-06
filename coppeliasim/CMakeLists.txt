cmake_minimum_required(VERSION 3.16)
project(coppelia_sim_slam)

find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs)

catkin_package(CATKIN_DEPENDS rospy roscpp std_msgs)