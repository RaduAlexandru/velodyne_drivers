
#include <ros/ros.h>
#include <ros/package.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "concatenater_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");


  namespace po = boost::program_options;

  po::options_description desc("Options");
  desc.add_options()
      ("help", "This help message")
      ("input_bag", po::value<std::string>(), "Input bagfile")
      ("output_bag", po::value<std::string>(), "Output bagfile")
      ("min_range", po::value<double>()->default_value(0.1), "Min range")
      ("max_range", po::value<double>()->default_value(130), "Max range")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv,desc,po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
      ROS_INFO_STREAM(desc);
      return 1;
  }
  if(!vm.count("input_bag"))
  {
    ROS_ERROR_STREAM("Concatenator: input bag file not given.");
    ROS_INFO_STREAM(desc);
    return -1;
  }
  std::string input_bag = vm["input_bag"].as<std::string>();
  if(!vm.count("output_bag"))
  {
    ROS_ERROR_STREAM("Concatenator: output bag file not given.");
    ROS_INFO_STREAM(desc);
    return -1;
  }
  std::string output_bag = vm["output_bag"].as<std::string>();
  double min_range = vm["min_range"].as<double>();
  double max_range = vm["max_range"].as<double>();

  rosbag::Bag bag_read;
  rosbag::Bag bag_write;
  bag_read.open(input_bag, rosbag::bagmode::Read);
  bag_write.open(output_bag, rosbag::bagmode::Write);

  rosbag::View view_all(bag_read);

  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan());
  // allocate a point cloud with same time and frame ID as raw data
  velodyne_rawdata::VPointCloud::Ptr cloud(new velodyne_rawdata::VPointCloud());
  cloud->height = 1;

  boost::shared_ptr<velodyne_rawdata::RawData> data(new velodyne_rawdata::RawData());
  std::string pkg_path = ros::package::getPath("velodyne_pointcloud");
  data->setupOffline(pkg_path + "/params/VLP16db.yaml", max_range, min_range);
  data->setParameters(min_range, max_range, 0.0, 0.0, false);

  int last_base_rotation = -1;
  int cut_angle = 0;

  BOOST_FOREACH(rosbag::MessageInstance const m, view_all)
  {
    if(m.getTopic() == "/velodyne_packets")
    {
      velodyne_msgs::VelodyneScan::ConstPtr scan_msg = m.instantiate<velodyne_msgs::VelodyneScan>();
      if(scan_msg != NULL)
      {
        for(int i = 0; i < scan_msg->packets.size(); i++)
        {
          scan->packets.push_back(scan_msg->packets[i]);

          data->unpack(scan_msg->packets[i], *cloud);

          // Extract base rotation of first block in packet
          std::size_t rot_data_pos = 100*0+2;
          int base_rotation = *( (u_int16_t*) (&scan_msg->packets[i].data[rot_data_pos]));
          // Handle overflow 35999->0
          if(base_rotation < last_base_rotation)
            last_base_rotation -= 36000;
          // Check if currently passing cut angle
          if(   last_base_rotation != -1
                && last_base_rotation < cut_angle
                && base_rotation >= cut_angle )
          {
            last_base_rotation = base_rotation;
            velodyne_msgs::VelodyneScanPtr out_scan(new velodyne_msgs::VelodyneScan);
            out_scan.swap(scan);

            pcl::PCLPointCloud2 pcl_pc2;
            pcl::toPCLPointCloud2(*cloud, pcl_pc2);
            sensor_msgs::PointCloud2 out_cloud;
            pcl_conversions::fromPCL(pcl_pc2, out_cloud);

            // reserve space for next run
            scan->packets.reserve(out_scan->packets.size());
            cloud->clear();
            cloud->height = 1;
            cloud->points.reserve(out_cloud.width * out_cloud.height);

            out_scan->header.stamp = out_scan->packets.back().stamp;
            out_scan->header.frame_id = scan_msg->header.frame_id;
            out_cloud.header.stamp = out_scan->packets.back().stamp;
            out_cloud.header.frame_id = "velodyne";

            // publish the accumulated packet message
            ROS_DEBUG_STREAM("Writing " << out_scan->packets.size()
                                           << " Velodyne packets, time: " << out_scan->header.stamp);
            bag_write.write(m.getTopic(), out_scan->header.stamp, out_scan);
            bag_write.write("/velodyne_points", out_cloud.header.stamp, out_cloud);
          }
          last_base_rotation = base_rotation;

        }
      }
    }
    else if(m.getTopic() == "/velodyne_points")
    {
      continue;
    }
    else
    {
      bag_write.write(m.getTopic(),m.getTime(),m);
    }
  }
  bag_read.close();
  bag_write.close();

  return 1;
}
