cmake_minimum_required(VERSION 2.8.3)
project(poine_orbslam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()


MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -mtune=native")


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
## Find catkin macros and libraries
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  compressed_depth_image_transport
  compressed_image_transport
  cv_bridge
  image_transport
  kinect2_bridge
  message_filters
  message_filters
  kinect2_bridge
  roscpp
  rostime
  rosbag
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package( G2O REQUIRED )
find_package(OpenMP)
find_package(Pangolin 0.1 REQUIRED)
find_package(PCL REQUIRED)
find_package(kinect2_bridge REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS})
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

LINK_DIRECTORIES(${VTK_LIBRARY_DIRS})
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

find_library(BOOST_LIBRARY
    NAMES libboost_system.so libboost_thread.so libboost_date_time.so libboost_iostreams.so libboost_filesystem.so libboost_serialization.so
    PATHS
    /usr/lib/x86_64-linux-gnu
    PATHS_SUFFIXES ${BOOST_PATH_SUFFIXES}    )

find_library(OPENNI_LIBRARY
             NAMES libOpenNI.so
             PATHS
             /usr/lib
             PATH_SUFFIXES ${OPENNI_PATH_SUFFIXES}
)

find_library(EXTRA_LIBRARY
    NAMES libstdc++.so
    PATHS
    /usr/lib/gcc/x86_64-linux-gnu/5
    PATH_SUFFIXES ${EXTRA_PATH_SUFFIXES}
    )
find_library(LIBG2O_LIBRARY
	NAMES libg2o_core.so 
	PATHS
	/usr/local/lib
	PATH_SUFFIXES ${LIBG2O_PATH_SUFFIXES}
)


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
include_directories(${Pangolin_INCLUDE_DIRS})
if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(WIN32)
  add_definitions(-Dlinux=0)
  add_definitions(-DWIN32_LEAN_AND_MEAN)
  add_definitions(-DNOMINMAX)
  set(ADDITIONAL_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
  set(EXTRA_WINDOWS_LIBS ${EXTRA_WINDOWS_LIBS} ws2_32 ${JPEG_LIBRARY})
else()
  add_definitions(-Dlinux=1)
endif()


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
#   Message1.msg
   Matrix.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
		
 )

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
## catkin specific  ##
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
#  LIBRARIES poine_orb
  CATKIN_DEPENDS 
  rosbag
 cv_bridge kinect2_bridge
#  cv_bridge kinect2_bridge
#  DEPENDS system_lib
	message_runtime
)

###########
## Build ##
###########
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)
## Specify additional locations of header files
include_directories(/usr/include/openni2)
include_directories(/usr/include/ni)
include_directories(/usr/include/qt4)
include_directories(/usr/local/include/opencv)
include_directories(/usr/include/qt4/Qt)
include_directories(/usr/include/qt4/QtCore)
include_directories(/usr/include/qt4/QtGui)
include_directories(/usr/include/qt4/QtOpenGL)
include_directories(usr/include)
include_directories(/usr/local/include)
include_directories(/usr/include/boost)
include_directories(/usr/include/eigen3)
include_directories(/usr/include/flann)
include_directories(/usr/include/qhull)
include_directories(/usr/local/include/g2o)
include_directories(/home/robooster/poine_ws/src/poine_orbslam/src/Thirdparty/g2o/g2o)
include_directories(/home/jinglun/orbslam_imu_ws/src/poine_orbslam/src/Thirdparty/g2o/g2o)
include_directories(${VTK_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${kinect2_bridge_INCLUDE_DIRS}
	${kinect2_bridge_INCLUDE_DIRS}
)
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/poine_orb.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/poine_orb_node.cpp)

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
add_executable(poine_orbslam src/poine_orbslam.cc)

add_executable(view_map src/view_map.cc)

# add_executable(ros_vio src/ros_vio.cc)

# add_executable(rgbd src/ros_rgbd.cc)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable(rgbd src/ros_rgbd.cc)

add_library(libofpoine_orbslam SHARED
src/Camera.cc
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/PangolinViewer.cc
src/pointcloudmapping.cc
src/imupose.cpp
src/imustate.cpp
src/Thirdparty/DBoW2/DUtils/Random.cpp
src/Thirdparty/DBoW2/DUtils/Timestamp.cpp
src/Thirdparty/DBoW2/DBoW2/BowVector.cpp
src/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp
src/Thirdparty/DBoW2/DBoW2/FORB.cpp
src/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp
src/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
src/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
src/Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
src/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
src/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
)

target_link_libraries(poine_orbslam
   #${LIBG2O_LIBRARY}
   ${catkin_LIBRARIES}
libofpoine_orbslam
    ${BOOST_LIBRARY}
    ${OPENNI_LIBRARY}
    ${VTK_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  ${kinect2_bridge_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${DBoW_LIBRARY}
${PROJECT_SOURCE_DIR}/src/Thirdparty/DBoW2/lib/libDBoW2.so 
${PROJECT_SOURCE_DIR}/src/Thirdparty/g2o/lib/libg2o.so
)

target_link_libraries(view_map
   #${LIBG2O_LIBRARY}
libofpoine_orbslam

    ${BOOST_LIBRARY}
   ${OPENNI_LIBRARY}
   ${VTK_LIBRARIES}
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
 ${PCL_LIBRARIES}
 ${kinect2_bridge_LIBRARIES}
 ${Pangolin_LIBRARIES}
 ${DBoW_LIBRARY}
${PROJECT_SOURCE_DIR}/src/Thirdparty/DBoW2/lib/libDBoW2.so 
${PROJECT_SOURCE_DIR}/src/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/lib/liblibofkinect2_orbslam.so
)

# target_link_libraries(ros_vio
#    #${LIBG2O_LIBRARY}
# libofpoine_orbslam
# # liblibofkinect2_orbslam.so
#     ${BOOST_LIBRARY}
#     ${OPENNI_LIBRARY}
#     ${VTK_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
# ${OpenCV_LIBS}
# ${EIGEN3_LIBS}
#   ${PCL_LIBRARIES}
#   ${kinect2_bridge_LIBRARIES}
#  ${Pangolin_LIBRARIES}
#   ${DBoW_LIBRARY}
# ${PROJECT_SOURCE_DIR}/src/Thirdparty/DBoW2/lib/libDBoW2.so 
# ${PROJECT_SOURCE_DIR}/src/Thirdparty/g2o/lib/libg2o.so
# ${PROJECT_SOURCE_DIR}/lib/liblibofkinect2_orbslam.so
# )

# target_link_libraries(rgbd
#    #${LIBG2O_LIBRARY}
# libofpoine_orbslam
#     ${BOOST_LIBRARY}
#     ${OPENNI_LIBRARY}
#     ${VTK_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
# ${OpenCV_LIBS}
# ${EIGEN3_LIBS}
#   ${PCL_LIBRARIES}
#   ${kinect2_bridge_LIBRARIES}
#  ${Pangolin_LIBRARIES}
#   ${DBoW_LIBRARY}
# ${PROJECT_SOURCE_DIR}/src/Thirdparty/DBoW2/lib/libDBoW2.so 
# ${PROJECT_SOURCE_DIR}/src/Thirdparty/g2o/lib/libg2o.so
# )

target_link_libraries(rgbd
   #${LIBG2O_LIBRARY}
	# ${PROJECT_SOURCE_DIR}/lib/liblibofpoine_orbslam
	libofpoine_orbslam
    ${BOOST_LIBRARY}
    ${OPENNI_LIBRARY}
    ${VTK_LIBRARIES}
    ${catkin_LIBRARIES}
 	${OpenCV_LIBRARIES}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
  	${PCL_LIBRARIES}
  	${kinect2_bridge_LIBRARIES}
 	${Pangolin_LIBRARIES}
  	${DBoW_LIBRARY}
	${PROJECT_SOURCE_DIR}/src/Thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/src/Thirdparty/g2o/lib/libg2o.so
)



target_link_libraries(poine_orbslam
   #${LIBG2O_LIBRARY}
	# ${PROJECT_SOURCE_DIR}/lib/liblibofpoine_orbslam
	libofpoine_orbslam
    ${BOOST_LIBRARY}
    ${OPENNI_LIBRARY}
    ${VTK_LIBRARIES}
    ${catkin_LIBRARIES}
 	${OpenCV_LIBRARIES}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
  	${PCL_LIBRARIES}
  	${kinect2_bridge_LIBRARIES}
 	${Pangolin_LIBRARIES}
  	${DBoW_LIBRARY}
	${PROJECT_SOURCE_DIR}/src/Thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/src/Thirdparty/g2o/lib/libg2o.so
)

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_poine_orb.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

# add_executable(rgbd_tum
# Examples/RGB-D/rgbd_tum.cc)
# target_link_libraries(rgbd_tum libofpoine_orbslam)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti
# Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti libofpoine_orbslam)

# add_executable(stereo_euroc
# Examples/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc libofpoine_orbslam)


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum
# Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum libofpoine_orbslam)

# add_executable(mono_kitti
# Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti libofpoine_orbslam)

# add_executable(mono_euroc
# Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc libofpoine_orbslam)

