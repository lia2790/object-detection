cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)#modulo del compilatore
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)#se è supportato
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")#aggiunge la macro alla compilazione
ELSEIF(COMPILER_SUPPORTS_CXX0X)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")#l'ERROR fa Piantare il cmake
ENDIF()



#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()
#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )


#### Eigen3, modify as needed
#SET(EIGEN3_INCLUDE_DIR "$ENV{HOME}/eigen")

#### Requires Eigen3
#IF (NOT EIGEN3_INCLUDE_DIR)
#    MESSAGE(FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF ()
#INCLUDE_DIRECTORIES(SYSTEM "${EIGEN3_INCLUDE_DIR}")





 
# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  geometry_msgs
  image_transport
  cv_bridge
  tf
  visualization_msgs
)




FIND_PACKAGE( MRPT REQUIRED base) # WARNING: Add all the MRPT libs used by your program: "gui", "obs", "slam",etc.





#find_package(catkin REQUIRED cmake_modules)
#find_package(Eigen REQUIRED)
#find_package(cmake_modules REQUIRED)
#find_package(eigen REQUIRED)
#INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
#include_directories(${EIGEN_INCLUDE_DIRS})



set(flycapHeader "/usr/include/flycapture")





find_library(FLYCAPTURE2 flycapture)

if(FLYCAPTURE2)
	message(STATUS ${FLYCAPTURE2})
else(FLYCAPTURE2)
   message(FATAL_ERROR "FlyCapture2 is needed for the Point Grey cameras.  Cannot continue.")
endif(FLYCAPTURE2)





FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



set(SOURCE1 src/ar_track/publisherROSimage.cpp
				src/ar_track/FlyCap.cpp)

set(SOURCE2 src/ar_track/publisherROSobj.cpp)

set(SOURCE3 src/ar_track/publisherROSKalman3.cpp)

set(SOURCE4 src/ar_track/Skew.cpp)

set(SOURCE5 src/ar_track/Stima_R_sensore.cpp)

set(SOURCE6 src/ar_track/TestFilter.cpp)

set(SOURCE7 src/ar_track/euleroQuat.cpp)



INCLUDE_DIRECTORIES( include
							${catkin_INCLUDE_DIRS}
							#${Boost_INCLUDE_DIRS}
							#${Eigen_INCLUDE_DIRS}
							${flycapHeader})#directory where there are the file.h






## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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

## Generate services in the 'srv' folder

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
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
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#	DEPENDS Eigen
)










###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#	${Eigen_INCLUDE_DIRS}
#)



#nome dell'eseguibile

ADD_EXECUTABLE(fly_cap_pub ${SOURCE1})

ADD_EXECUTABLE(obj_pose_in_the_world ${SOURCE2})

ADD_EXECUTABLE(Kalman ${SOURCE3})

ADD_EXECUTABLE(Skew_velocity ${SOURCE4})

ADD_EXECUTABLE(Stima ${SOURCE5})

ADD_EXECUTABLE(TestFilter ${SOURCE6})

ADD_EXECUTABLE(EULquat ${SOURCE7})




#libreria da linkare

TARGET_LINK_LIBRARIES(fly_cap_pub ${catkin_LIBRARIES} ${FLYCAPTURE2}) #${Boost_LIBRARIES})

TARGET_LINK_LIBRARIES(obj_pose_in_the_world ${catkin_LIBRARIES} ${MRPT_LIBS})

TARGET_LINK_LIBRARIES(Kalman ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

TARGET_LINK_LIBRARIES(Skew_velocity ${catkin_LIBRARIES} ${MRPT_LIBS} ${Eigen_LIBRARIES})

TARGET_LINK_LIBRARIES(Stima ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

TARGET_LINK_LIBRARIES(TestFilter ${catkin_LIBRARIES} ${MRPT_LIBS})

TARGET_LINK_LIBRARIES(EULquat ${catkin_LIBRARIES} ${MRPT_LIBS})



# BUILD SERVICE SERVER
# %Tag(SRVCLIENT)%
# ADD_EXECUTABLE(Start&StopServer src/detection_marker.cpp)
# TARGET_LINK_LIBRARIES(Start&StopServer ${catkin_LIBRARIES})
# ADD_DEPENDENCIES(Start&StopServer object_detection)

# %EndTag(SRVCLIENT)%
# %EndTag(FULLTEXT)%




## Declare a C++ library
# add_library(object_detection
#   src/${PROJECT_NAME}/object_detection.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(object_detection_node src/object_detection_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(object_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(object_detection_node
#   ${catkin_LIBRARIES}
# )



















#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS object_detection object_detection_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_detection.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
