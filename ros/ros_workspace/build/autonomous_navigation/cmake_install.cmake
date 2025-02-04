# Install script for directory: /home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/src/autonomous_navigation

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_navigation/msg" TYPE FILE FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/src/autonomous_navigation/msg/Vector3Array.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_navigation/cmake" TYPE FILE FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/autonomous_navigation-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/devel/include/autonomous_navigation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/devel/share/roseus/ros/autonomous_navigation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/devel/share/common-lisp/ros/autonomous_navigation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/devel/share/gennodejs/ros/autonomous_navigation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/devel/lib/python3/dist-packages/autonomous_navigation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/devel/lib/python3/dist-packages/autonomous_navigation")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/autonomous_navigation.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_navigation/cmake" TYPE FILE FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/autonomous_navigation-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_navigation/cmake" TYPE FILE FILES
    "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/autonomous_navigationConfig.cmake"
    "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/autonomous_navigationConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/autonomous_navigation" TYPE FILE FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/src/autonomous_navigation/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/autonomous_navigation" TYPE PROGRAM FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/path_publisher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/autonomous_navigation" TYPE PROGRAM FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/robot_controller.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/autonomous_navigation" TYPE PROGRAM FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/robot_controller_A.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/autonomous_navigation" TYPE PROGRAM FILES "/home/lab_rob_shared/lab_robotica_g3/ros/ros_workspace/build/autonomous_navigation/catkin_generated/installspace/obstacle_detection.py")
endif()

