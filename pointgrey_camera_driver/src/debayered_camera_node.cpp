#include <nodelet/loader.h>
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "debayered_camera_node");

  nodelet::Loader manager;  // Don't bring up the manager ROS API
  nodelet::V_string nargv;
  nodelet::M_string remap(ros::names::getRemappings());
  remap["image_raw"] = ros::names::resolve("~/image_raw");

  std::string nodelet_name = ros::this_node::getName();
  
  manager.load(nodelet_name + "PointGreyCameraNodelet",
               "pointgrey_camera_driver/PointGreyCameraNodelet", remap, nargv);
  ROS_INFO_STREAM("Started " << nodelet_name << "/PointGreyCameraNodelet"
                             << " nodelet.");

  manager.load(nodelet_name + "/debayer", "image_proc/debayer", remap, nargv);
  ROS_INFO_STREAM("Started " << nodelet_name << "/debayer"
                             << " nodelet.");

  ros::spin();
  return 0;
}