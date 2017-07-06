
#include <ros/ros.h>
#include "concatenate.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "concatenate_node");

  // create conversion class, which subscribes to raw data
  velodyne_pointcloud::Concatenate concatenate(ros::NodeHandle(),
                                                ros::NodeHandle("~"));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
