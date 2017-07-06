
#ifndef _VELODYNE_POINTCLOUD_CONCATENATE_H_
#define _VELODYNE_POINTCLOUD_CONCATENATE_H_ 1

#include <ros/ros.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud
{
  class Concatenate
  {
  public:

    Concatenate(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Concatenate() {}

  private:

    void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg);

    velodyne_msgs::VelodyneScanPtr scan_;
    int number_of_scans_in_full_rotation_;
    int cut_angle_;
    int last_base_rotation_;

    bool has_finished_rotation_once_;

    ros::Subscriber velodyne_scan_;
    ros::Publisher output_;
  };

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONCATENATE_H_
