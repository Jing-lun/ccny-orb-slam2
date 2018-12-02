<<<<<<< HEAD
# Install script for directory: /home/all3n/catkin_ws/src/poine_orbslam/src/Thirdparty/DBoW2
=======
# Install script for directory: /home/jinglun/orbslam_imu_ws/src/poine_orbslam/src/Thirdparty/DBoW2
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
<<<<<<< HEAD
file(WRITE "/home/all3n/catkin_ws/src/poine_orbslam/src/Thirdparty/DBoW2/build/${CMAKE_INSTALL_MANIFEST}"
=======
file(WRITE "/home/jinglun/orbslam_imu_ws/src/poine_orbslam/src/Thirdparty/DBoW2/build/${CMAKE_INSTALL_MANIFEST}"
>>>>>>> d44933e8356f6014b6c205e747bf5c9be8dd54d6
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
