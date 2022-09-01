cmake_minimum_required(VERSION 3.1)
project(open_loop_controller)

find_package(catkin
        # ROS components
        REQUIRED COMPONENTS
        rospy
        std_msgs)

catkin_python_setup()