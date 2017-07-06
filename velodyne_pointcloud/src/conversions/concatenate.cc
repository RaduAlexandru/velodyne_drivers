
#include "concatenate.h"

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Concatenate::Concatenate(ros::NodeHandle node, ros::NodeHandle private_nh):
    number_of_scans_in_full_rotation_(0),
    cut_angle_(0),
    last_base_rotation_(-1),
    has_finished_rotation_once_(false),
    scan_(new velodyne_msgs::VelodyneScan())
  {
    output_ = node.advertise<velodyne_msgs::VelodyneScan>("velodyne_concatenated_packets", 10);
    
    velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Concatenate::processScan, (Concatenate *) this,
            ros::TransportHints().tcpNoDelay(true));
  }

  /** @brief Callback for raw scan messages. */
  void
  Concatenate::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)
    {
      last_base_rotation_ = -1;
      scan_.reset();
      return;
    }

    if(has_finished_rotation_once_)
      scan_->packets.reserve(number_of_scans_in_full_rotation_);

    for(int i = 0; i < scanMsg->packets.size(); i++)
    {
      scan_->packets.push_back(scanMsg->packets[i]);

      // Extract base rotation of first block in packet
      std::size_t rot_data_pos = 100*0+2;
      int base_rotation = *( (u_int16_t*) (&scanMsg->packets[i].data[rot_data_pos]));
      // Handle overflow 35999->0
      if(base_rotation<last_base_rotation_)
        last_base_rotation_-=36000;
      // Check if currently passing cut angle
      if(   last_base_rotation_ != -1
            && last_base_rotation_ < cut_angle_
            && base_rotation >= cut_angle_ )
      {
        // Cut angle passed, one full revolution collected
        has_finished_rotation_once_ = true;
        number_of_scans_in_full_rotation_ = scan_->packets.size();

        last_base_rotation_ = base_rotation;
        velodyne_msgs::VelodyneScanPtr out_scan(new velodyne_msgs::VelodyneScan);
        out_scan.swap(scan_);

        out_scan->header.stamp = out_scan->packets.back().stamp;
        out_scan->header.frame_id = scanMsg->header.frame_id;
        // publish the accumulated packet message
        ROS_DEBUG_STREAM("Publishing " << out_scan->packets.size()
                                       << " Velodyne packets, time: " << out_scan->header.stamp);
        output_.publish(out_scan);
      }
      last_base_rotation_ = base_rotation;

    }
  }

} // namespace velodyne_pointcloud
