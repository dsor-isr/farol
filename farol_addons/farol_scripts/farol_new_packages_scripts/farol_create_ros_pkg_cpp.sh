#!/bin/bash

# Description: bash script for creating a ros cpp package according with the farol stack best practices

# @.@ assign first received argument to variable PKG_NAME 
if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash farol_create_ros_pkg_cpp.sh fiic_awesome_cpp"
fi

# @.@ assign first received argument to variable PKG_NAME
PKG_NAME_IN=${1,,}
PKG_NAME=$(echo "$PKG_NAME_IN" | sed -r 's/_([a-z])/\U\1/g')

# @.@ Give some author love
AUTHOR_NAME="#DSORTeam"
AUTHOR_EMAIL="dsor.isr@gmail.com"
MAINTAINER_NAME="#DSORTeam"
MAINTAINER_EMAIL="dsor.isr@gmail.com"

# @.@ setup file content
CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 3.1.0)
project(${PKG_NAME_IN})

find_package(catkin
 # ROS components
 REQUIRED COMPONENTS
 std_msgs 
 farol_msgs
 roscpp

 # farol_gimmicks_library
 farol_gimmicks_library
)

catkin_package(
 CATKIN_DEPENDS
)

add_compile_options(-std=c++17 -Wall -O3) 

include_directories(
 include/${PKG_NAME_IN}_ros
 include/${PKG_NAME_IN}_algorithms
 \${catkin_INCLUDE_DIRS}
)

add_executable(\${PROJECT_NAME}_node src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.cpp src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.cpp)
add_dependencies(\${PROJECT_NAME}_node \${catkin_EXPORTED_TARGETS})
target_link_libraries(\${PROJECT_NAME}_node \${catkin_LIBRARIES})
"


PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>
<package format=\"2\">
 <name>${PKG_NAME_IN}</name>
 <version>1.0.0</version>
 <description>
 ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 </description>

 <license>MIT</license>

 <author email=\"dsor.isr@gmail.com\">#DSORTeam</author>
 <maintainer email=\"dsor.isr@gmail.com\">#DSORTeam</maintainer>

 <buildtool_depend>catkin</buildtool_depend>

 <!-- Add here ROS dependencies -->
 <depend>roscpp</depend>
 <depend>farol_msgs</depend>

</package>
"


CONFIG_YAML_CONTENT="node_frequency: 2
"


README_CONTENT="${PKG_NAME} documentation
"


LAUNCH_FILE_CONTENT="<?xml version=\"1.0\"?>
<launch>

 <!-- small description about your node -->
 
 <node pkg=\"${PKG_NAME_IN}\" type=\"${PKG_NAME_IN}_node\" name=\"${PKG_NAME^}Node\" respawn=\"false\" output=\"screen\">
	<rosparam command=\"load\" file=\"\$(find ${PKG_NAME_IN})/config/config_${PKG_NAME_IN}.yaml\"/>
</node>

</launch>
"


MY_TEST_CONTENT="
"

ROS_NODE_CONTENT="/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include \"${PKG_NAME^}Node.h\"
#include \"${PKG_NAME^}Algorithm.h\"

// @.@ Constructor
${PKG_NAME^}Node::${PKG_NAME^}Node(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

}

// @.@ Destructor
${PKG_NAME^}Node::~${PKG_NAME^}Node() {

  // +.+ shutdown publishers


  // +.+ shutdown subscribers


  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void ${PKG_NAME^}Node::loadParams() {
  ROS_INFO(\"Load the ${PKG_NAME^}Node parameters\");

  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, \"node_frequency\", 5);

}


// @.@ Member helper function to set up subscribers
void ${PKG_NAME^}Node::initializeSubscribers() {
  ROS_INFO(\"Initializing Subscribers for ${PKG_NAME^}Node\");
  
}


// @.@ Member helper function to set up publishers
void ${PKG_NAME^}Node::initializePublishers() {
  ROS_INFO(\"Initializing Publishers for ${PKG_NAME^}Node\");

}


// @.@ Member helper function to set up services
void ${PKG_NAME^}Node::initializeServices() {
  ROS_INFO(\"Initializing Services for ${PKG_NAME^}Node\");

}


// @.@ Member helper function to set up the timer
void ${PKG_NAME^}Node::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &${PKG_NAME^}Node::timerIterCallback, this);
}


// @.@ Where the magic should happen.
void ${PKG_NAME^}Node::timerIterCallback(const ros::TimerEvent &event) {

}


/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, \"${PKG_NAME_IN}_node\"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private(\"~\");

  ROS_INFO(\"main: instantiating an object of type ${PKG_NAME^}Node\");

  // +.+ instantiate an ${PKG_NAME^}Node class object and pass in pointers to nodehandle public and private for constructor to use
  ${PKG_NAME^}Node ${PKG_NAME_IN}(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
"

ROS_INDEPENDENT_CLASS_CONTENT="//TODO: Algorithm Code Here\n\n
"
ROS_INDEPENDENT_CLASS_H="//TODO: Algorithm declarations Here\n\n
"
ROS_NODE_H="/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_${PKG_NAME^^}NODE_H
 #define CATKIN_WS_${PKG_NAME^^}NODE_H

 //some generically useful stuff to include...
 #include <std_msgs/String.h>
 #include <vector>
 #include <ros/ros.h> 

 #include <farol_gimmicks_library/FarolGimmicks.h>

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class ${PKG_NAME^}Node {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	${PKG_NAME^}Node(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~${PKG_NAME^}Node();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

 	// @.@ Subsctibers


 	// @.@ Publishers


 	// @.@ Timer
 	ros::Timer timer_;           ///< ROS Timer

  // @.@ Parameters from Yaml
  double p_node_frequency_;   ///< node frequency

 	// @.@ Problem variables

 	

  // @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Load parameters from parameter server 
   */
  /* -------------------------------------------------------------------------*/
 	void loadParams();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize ROS node Subscribers
   */
  /* -------------------------------------------------------------------------*/
  void initializeSubscribers();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Initialize ROS node Publishers
   */
  /* -------------------------------------------------------------------------*/
 	void initializePublishers();

 
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Initialize ROS node Services
   */
  /* -------------------------------------------------------------------------*/
 	void initializeServices();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize ROS node Timer  
   */
  /* -------------------------------------------------------------------------*/
 	void initializeTimer();


 	
  // @.@ Callbacks declaration
 
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerIterCallback(const ros::TimerEvent& event);



  // @.@ Services declaration



  // @.@ Member helper functions


};
#endif //CATKIN_WS_CONTROLNODE_H
"

###############################################################
# @.@ create folder structure                                 #  
###############################################################
mkdir ${PKG_NAME_IN} 
cd ${PKG_NAME_IN}
mkdir -p include/${PKG_NAME_IN}_ros
mkdir -p include/${PKG_NAME_IN}_algorithms
mkdir -p config
mkdir -p docs
mkdir -p launch
mkdir -p src/${PKG_NAME_IN}_ros
mkdir -p src/${PKG_NAME_IN}_algorithms
mkdir -p test

################################################################
# @.@ create file structure                                    # 
################################################################
touch CMakeLists.txt
touch include/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.h
touch include/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.h
touch package.xml
touch config/config_${PKG_NAME_IN}.yaml
touch docs/README.md
touch launch/${PKG_NAME_IN}.launch
touch src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.cpp
touch src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.cpp
touch test/${PKG_NAME_IN}_test.cpp

################################################################
# @.@ fill files with information from parameter section       #
################################################################
echo -e $CMAKELISTS_CONTENT$  > CMakeLists.txt
sed -i '$d' CMakeLists.txt 
echo -e $PACKAGE_XML_CONTENT$ > package.xml
sed -i '$d' package.xml 
echo -e $CONFIG_YAML_CONTENT$ > config/config_${PKG_NAME_IN}.yaml
sed -i '$d' config/config_${PKG_NAME_IN}.yaml
echo -e $README_CONTENT$ > docs/README.md
sed -i '$d' docs/README.md 
echo -e $LAUNCH_FILE_CONTENT$ > launch/${PKG_NAME_IN}.launch 
sed -i '$d' launch/${PKG_NAME_IN}.launch
echo -e $MY_TEST_CONTENT$ > test/${PKG_NAME_IN}_test.cpp
sed -i '$d' test/${PKG_NAME_IN}_test.cpp
echo -e $ROS_NODE_CONTENT$ > src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.cpp
sed -i '$d' src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.cpp
echo -e $ROS_NODE_H$ > include/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.h
sed -i '$d' include/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.h
echo -e $ROS_INDEPENDENT_CLASS_CONTENT$ > src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.cpp
sed -i '$d' src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.cpp
echo -e $ROS_INDEPENDENT_CLASS_H$ > include/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.h
sed -i '$d' include/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.h

################################################################
# @.@ Make or build the workspace, you decide                  #
################################################################
farol_cbt

################################################################
# @.@ Unset the created local variables of this file           #
################################################################
unset PKG_NAME
unset AUTHOR_NAME
unset AUTHOR_EMAIL
unset MAINTAINER_NAME
unset MAINTAINER_EMAIL
unset CMAKELISTS_CONTENT
unset PACKAGE_XML_CONTENT
unset CONFIG_YAML_CONTENT
unset README_CONTENT
unset LAUNCH_FILE_CONTENT
unset MY_TEST_CONTENT
unset ROS_NODE_CONTENT
unset ROS_NODE_H
unset ROS_INDEPENDENT_CLASS_CONTENT

################################################################
# @.@ Source bashrc and echo an ecouriging goodbye message     #
################################################################
source ~/.bashrc
echo "${PKG_NAME_IN} ros cpp package created, you are now inside it. Let the fun begin..." 
unset PKG_NAME_IN