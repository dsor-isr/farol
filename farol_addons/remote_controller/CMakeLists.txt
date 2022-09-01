cmake_minimum_required(VERSION 3.1)
project(remote_controller)

find_package(catkin
        # ROS components
        REQUIRED COMPONENTS
        rospy
        auv_msgs
        std_msgs)