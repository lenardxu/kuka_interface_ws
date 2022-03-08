cmake_minimum_required(VERSION 3.0.2)
project(process_msg_ki)

# set the env variable for the project name to be used by code
set( NAMEOFPROJECT ${PROJECT_NAME} )
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Specify the configuration file from which the header file will be generated
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
message("PROJECT_BINARY_DIR = ${PROJECT_BINARY_DIR}")
configure_file (ProcessMsgKIConfig.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ProcessMsgKIConfig.h @ONLY)
#configure_file (ProcessMsgKIConfig.h.in ProcessMsgKIConfig.h @ONLY)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
###################################################################################
## Boost is already built in ROS, thus no need to use other built package for it ##
###################################################################################
#set( BOOST_ROOT "C:\\DevSoft\\boost\\boost_1_75_0" )
#set( BOOST_LIBRARIES "C:\\DevSoft\\boost\\boost_1_75_0\\stage\\lib" )
#set( Boost_USE_STATIC_LIBS ON )
#find_package( Boost 1.73.0 REQUIRED COMPONENTS system program_options date_time )
find_package( Boost REQUIRED COMPONENTS system program_options date_time )
message( STATUS
        "Boost_FOUND: ${Boost_FOUND}, Boost_VERSION:${Boost_VERSION},
        Boost_INCLUDE_DIRS:${Boost_INCLUDE_DIRS},
        Boost_LIBRARIES:${Boost_LIBRARIES}" )

#########################################################################################
## For catkin/CMakeLists.txt, the style of typing path should be consistent with Linux ##
#########################################################################################
######################Correct configs for xerces c++#####################################
#set ( XercesC_INCLUDE_DIR "D:/SUMOLibraries-main/xerces-c-3.2.3/src" )
#set ( XercesC_LIBRARY "D:/SUMOLibraries-main/xerces-c-3.2.3/lib/xerces-c_3.lib" )
#find_package( XercesC REQUIRED )
## Check the status after using find_package function
#message( STATUS "XercesC_FOUND: ${XercesC_FOUND}, XercesC_VERSION:${XercesC_VERSION},
#    XercesC_INCLUDE_DIRS:${XercesC_INCLUDE_DIRS},
#    XercesC_INCLUDE_DIR: ${XercesC_INCLUDE_DIR},
#    XercesC_LIBRARIES:${XercesC_LIBRARIES},
#    XercesC_LIBRARY: ${XercesC_LIBRARY}" )
#########################################################################################


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES process_msg_ki
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs
   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/process_msg_ki.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
# The following activated block (only for testing) serves for generating dummy robot msg and storing it into the shared
# memory so that process_robot_msg_node can access and operate on it
add_executable( generate_robot_msg_test
        src/generate_robot_msg_shm_first_test.cpp )
target_include_directories( generate_robot_msg_test
        PRIVATE ${Boost_INCLUDE_DIRS} )
target_link_libraries( generate_robot_msg_test
        ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

# The following commented block serves for udp server implementation with parsing xml msg using xerces-c++ library
# This way of parsing xml msg can be remained in case the msg gets more complex
#add_executable( kuka_interface_server_node
#        src/kuka_server_win.cpp src/parser.cpp )
#target_include_directories( kuka_interface_server_node
#        PRIVATE ${Boost_INCLUDE_DIRS} ${XercesC_INCLUDE_DIR} )
#target_link_libraries( kuka_interface_server_node
#        ${Boost_LIBRARIES} ${XercesC_LIBRARY} ${catkin_LIBRARIES} )
# The following activated block serves for udp server implementation with parsing xml msg using regex library (STL)
add_executable( kuka_interface_server_node
        src/kuka_server_real_time.cpp )
target_include_directories( kuka_interface_server_node
        PRIVATE ${Boost_INCLUDE_DIRS} )
target_link_libraries( kuka_interface_server_node
        ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

# The following activated block serves for processing the robot msg in shared memory and then publishing it
add_executable( process_robot_msg_node
        src/process_robot_msg_shm_first.cpp )
target_include_directories( process_robot_msg_node
        PRIVATE ${Boost_INCLUDE_DIRS} )
target_link_libraries( process_robot_msg_node
        ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

# The following activated block serves for subscribing to the ros msg over r_ist topic and then publishing dummy control signal
add_executable( controller_node
        src/dummy_controller.cpp )
#target_include_directories( controller_node
#        PRIVATE ${Boost_INCLUDE_DIRS} )
target_link_libraries( controller_node ${catkin_LIBRARIES} )

# The following activated block serves for subscribing to the robot msg over control_signal topic and then saving it to shm
add_executable( subscribe_ctrl_signal_node
        src/subscribe_ctrl_signal_shm_second.cpp )
target_include_directories( subscribe_ctrl_signal_node
        PRIVATE ${Boost_INCLUDE_DIRS} )
target_link_libraries( subscribe_ctrl_signal_node
        ${Boost_LIBRARIES} ${catkin_LIBRARIES} )


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install( TARGETS generate_robot_msg_test kuka_interface_server_node process_robot_msg_node controller_node subscribe_ctrl_signal_node
         RUNTIME DESTINATION lib/${PROJECT_NAME} )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install( DIRECTORY launch/
         DESTINATION share/${PROJECT_NAME}/launch )
install( DIRECTORY test/test_data/
         DESTINATION share/${PROJECT_NAME} )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_process_msg_ki.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
