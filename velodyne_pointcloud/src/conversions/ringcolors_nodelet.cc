/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "colors.h"

namespace velodyne_pointcloud
{
  class RingColorsNodelet: public nodelet::Nodelet
  {
  public:

    RingColorsNodelet() {}
    ~RingColorsNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<RingColors> colors_;
  };

  /** @brief Nodelet initialization. */
  void RingColorsNodelet::onInit()
  {
    colors_.reset(new RingColors(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne_pointcloud


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
//export_class is for ros_melodic, declare_class is for ros_kinetic
PLUGINLIB_EXPORT_CLASS(velodyne_pointcloud::RingColorsNodelet, nodelet::Nodelet);
//PLUGINLIB_DECLARE_CLASS(velodyne_pointcloud, RingColorsNodelet,
//                        velodyne_pointcloud::RingColorsNodelet, nodelet::Nodelet);
