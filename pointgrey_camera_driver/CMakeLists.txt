cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_camera_driver)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp nodelet sensor_msgs wfov_camera_msgs
  image_exposure_msgs camera_info_manager image_transport
  dynamic_reconfigure diagnostic_updater)

generate_dynamic_reconfigure_options(
  cfg/PointGrey.cfg
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS roscpp nodelet
)

# If the user has manually installed flycapture, use the system path version. If not,
# download it. We can't resolve this dependency using the usual rosdep means because
# the Point Grey EULA prohibits redistributing the headers or the packages which
# contains them. Fortunately, we can download the archive directly from their
# website during this build process.
find_library(POINTGREY_LIB flycapture)
if(NOT POINTGREY_LIB)
  message(STATUS "libflycapture not found in system library path")
  include(cmake/DownloadFlyCap.cmake)
  download_flycap(POINTGREY_LIB POINTGREY_INCLUDE_DIR)
  message(STATUS "libflycapture library: ${POINTGREY_LIB}")
  message(STATUS "libflycapture include: ${POINTGREY_INCLUDE_DIR}")
  include_directories(${POINTGREY_INCLUDE_DIR})
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(TimeSyncLib src/TimeSyncEKF.cpp)
target_link_libraries(TimeSyncLib ${catkin_LIBRARIES})

add_library(PointGreyCamera src/PointGreyCamera.cpp)
target_link_libraries(PointGreyCamera ${POINTGREY_LIB} ${catkin_LIBRARIES} TimeSyncLib)
add_dependencies(PointGreyCamera ${PROJECT_NAME}_gencfg)

add_library(PointGreyCameraNodelet src/nodelet.cpp)
target_link_libraries(PointGreyCameraNodelet PointGreyCamera ${catkin_LIBRARIES})

add_library(PointGreyStereoCameraNodelet src/stereo_nodelet.cpp)
target_link_libraries(PointGreyStereoCameraNodelet PointGreyCamera ${catkin_LIBRARIES})

add_executable(pointgrey_camera_node src/node.cpp)
target_link_libraries(pointgrey_camera_node PointGreyCamera ${catkin_LIBRARIES})
set_target_properties(pointgrey_camera_node
                      PROPERTIES OUTPUT_NAME camera_node PREFIX "")

add_executable(pointgrey_stereo_node src/stereo_node.cpp)
target_link_libraries(pointgrey_stereo_node PointGreyCamera ${catkin_LIBRARIES})
set_target_properties(pointgrey_stereo_node
                      PROPERTIES OUTPUT_NAME stereo_node PREFIX "")

add_executable(pointgrey_list_cameras src/list_cameras.cpp)
target_link_libraries(pointgrey_list_cameras PointGreyCamera ${catkin_LIBRARIES})
set_target_properties(pointgrey_list_cameras
                      PROPERTIES OUTPUT_NAME list_cameras PREFIX "")

add_executable(debayered_camera_node src/debayered_camera_node.cpp)
target_link_libraries(debayered_camera_node PointGreyCamera ${catkin_LIBRARIES})
set_target_properties(debayered_camera_node
                      PROPERTIES OUTPUT_NAME debayered_camera_node PREFIX "")

install(TARGETS
  PointGreyCamera
  PointGreyCameraNodelet
  PointGreyStereoCameraNodelet
  pointgrey_camera_node
  pointgrey_stereo_node
  pointgrey_list_cameras
  debayered_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Redistributing the flycapture .so file is permitted by the SDK EULA:
# http://www.ptgrey.com/support/kb/data/PGR-FlyCap-SDK-LA.pdf
install(FILES ${POINTGREY_LIB} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/bumblebee.launch)
  roslaunch_add_file_check(launch/camera.launch)
  roslaunch_add_file_check(launch/stereo.launch)

  find_package(roslint REQUIRED)
  roslint_cpp()
endif()
