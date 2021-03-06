cmake_minimum_required(VERSION 2.8.3)
project(move4d_ros_lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
FIND_PACKAGE (PkgConfig REQUIRED)

set (CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX)
  ELSE(APPLE)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  pluginlib
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

find_package(move3d REQUIRED)
FIND_PACKAGE(move4d REQUIRED)

#PKG_CHECK_MODULES(MOVE3D REQUIRED libmove3d)
#INCLUDE_directories(${MOVE3D_INCLUDE_DIRS})
#LINK_DIRECTORIES(${MOVE3D_LIBRARY_DIRS} )

SET(LIBS ${LIBS} move4d move3d)
# find_library(MOVE4D_VISIBILITY_PLANNER move4d-visibility-grid)
# if(NOT MOVE4D_VISIBILITY_PLANNER)
#     message(SEND_ERROR "move4d-visibility-grid library (plugin) not found")
# else()
#     message("link with ${MOVE4D_VISIBILITY_PLANNER}")
# endif()

SET(MOVE4D_PLUGINS_LIBRARIES ${MOVE4D_PLUGINS_LIBRARIES})

include_directories(${INCLUDE_DIRS})

## Generate services in the 'srv' folder
add_service_files(FILES
    SaveScenario.srv
)
generate_messages()

add_definitions(-DUSE_LIGHT_PLANNER -DUSE_GRASP_PLANNING)


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
  INCLUDE_DIRS include
  LIBRARIES move4d_ros_lib move4d_human_updater_default_plugins move4d_human_updater_hand_head_plugin
  CATKIN_DEPENDS message_runtime
  #DEPENDS MOVE3D-PLANNERS libmove3d
  DEPENDS Eigen3 Boost
  #INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${INCLUDE_DIRS}
    )

configure_file(cmake/templates/plugins.cpp.in src/plugins.cpp @ONLY)
## Declare a C++ library
add_library(move4d_ros_lib
  src/move4d_ros_lib.cpp
  src/scenemanager.cpp
  src/savescenariosrv.cpp
  src/jointstateupdater.cpp
  src/tfupdater.cpp
  src/humanmgr.cpp
  src/conversions.cpp
  src/robots_tf_publisher.cpp
  src/tools.cpp
  src/logging.cpp
  src/plugins.cpp
)

add_library(move4d_human_updater_default_plugins
  src/plugins/single_joint_2d_updater.cpp
  src/plugins/gaze_updater.cpp
)

add_library(move4d_human_updater_hand_head_plugin
  src/plugins/head_hand_updater.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(move4d_ros_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(move4d_ros_lib_node src/move4d_ros_lib_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(move4d_ros_lib_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(move4d_ros_lib
  ${catkin_LIBRARIES}
  ${LIBS}
)
target_link_libraries(move4d_human_updater_default_plugins
  ${catkin_LIBRARIES}
  ${LIBS}
)
target_link_libraries(move4d_human_updater_hand_head_plugin
  ${catkin_LIBRARIES}
  ${LIBS}
)

message("catkin_LIBRARIES: ${catkin_LIBRARIES}")
message("catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
message("INCLUDE_DIRS: ${INCLUDE_DIRS}")

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
install(TARGETS move4d_ros_lib move4d_ros_lib move4d_human_updater_default_plugins move4d_human_updater_hand_head_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
    default_updaters.xml
  # myfile1
  # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_move4d_ros_lib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
