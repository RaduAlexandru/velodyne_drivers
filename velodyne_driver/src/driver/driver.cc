/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <velodyne_msgs/VelodyneScan.h>

#include "driver.h"

#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>

namespace velodyne_driver
{

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("velodyne"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("64E"));
  
  std::string model_full_name;
  if ((config_.model == "64E_S2") || 
      (config_.model == "64E_S2.1"))    // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
      packet_rate_ = 3472.17;            // 1333312 / 384
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "64E")
    {
      packet_rate_ = 2600.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "32E")
    {
      packet_rate_ = 1808.0;
      model_full_name = std::string("HDL-") + config_.model;
    }
  else if (config_.model == "VLP16")
    {
      packet_rate_ = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
      model_full_name = "VLP-16";
    }
  else
    {
      ROS_ERROR_STREAM("unknown Velodyne LIDAR model: " << config_.model);
      packet_rate_ = 2600.0;
    }
  std::string deviceName(std::string("Velodyne ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  setSpinRate(config_.rpm);
  ROS_INFO_STREAM(deviceName << " rotating at " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate_ / frequency);
  private_nh.getParam("npackets", config_.npackets);
  ROS_INFO_STREAM("publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));
  
  double cut_angle;
  private_nh.param("cut_angle", cut_angle, -0.01);
  config_.cut_angle = int(cut_angle*100);
  ROS_INFO_STREAM("cat_angle: " << cut_angle);
  
  int udp_port;
  private_nh.param("port", udp_port, (int) DATA_PORT_NUMBER);

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_driver::
    VelodyneNodeConfig> > (private_nh);
  dynamic_reconfigure::Server<velodyne_driver::VelodyneNodeConfig>::
    CallbackType f;
  f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate_/config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("velodyne_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             0.1, 10),
                                        TimeStampStatusParam()));

  // open Velodyne input device or file
  if (dump_file != "")                  // have PCAP file?
    {
      // read data from packet capture file
      input_.reset(new velodyne_driver::InputPCAP(private_nh, udp_port,
                                                  packet_rate_, dump_file));
    }
  else
    {
      // read data from live socket
      input_.reset(new velodyne_driver::InputSocket(private_nh, udp_port));
    }

  // raw packet output topic
  output_ =
    node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);
    
    
    spin_rate_server_ = node.advertiseService("spin_rate", &VelodyneDriver::spinRateServiceCall, this);
    
}

bool VelodyneDriver::spinRateServiceCall(velodyne_msgs::LaserSpeed::Request& req,
                         velodyne_msgs::LaserSpeed::Response& res)
{
  int speed = req.speed;
  if (speed > 1200)
    speed = 1200;
  else if (speed < 300)
    speed = 300;
  res.speed = speed;
  
  ros::NodeHandle nh("~");
  nh.setParam(LASER_SPEED_PARAMETER, speed);
  
  config_.rpm = speed;
  setSpinRate(config_.rpm);
  ROS_INFO_STREAM("setting velodyne spinrate to " << config_.rpm << " RPM");
  double frequency = (config_.rpm / 60.0);     // expected Hz rate
  
  // default number of packets for each scan is a single revolution
  // (fractions rounded up)
  config_.npackets = (int) ceil(packet_rate_ / frequency);
  return true;
}


bool VelodyneDriver::setSpinRate(int rate)
{
  boost::asio::io_service io_service;
  
  boost::asio::ip::tcp::resolver resolver(io_service);
  boost::asio::ip::tcp::resolver::query query("192.168.1.201",  "80"); // "http");
  boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
  boost::asio::ip::tcp::resolver::iterator end;
  
  boost::asio::ip::tcp::socket socket(io_service);
  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && endpoint_iterator != end)
  {
    socket.close();
    socket.connect(*endpoint_iterator++, error);
  }
  if (error)
    throw boost::system::system_error(error);
  
  boost::asio::streambuf request;
  std::ostream request_stream(&request);
  
  std::stringstream ss_to_send;
  ss_to_send << "rpm=";
  ss_to_send << rate; 
  ss_to_send << "\r\n";
  
  request_stream << "POST /cgi/setting HTTP/1.1\r\n";
  request_stream << "Host: localhost:8080\r\n";
  request_stream << "Connection: keep-alive\r\n";
  request_stream << "Content-Length: " <<  ss_to_send.str().length() << "\r\n";
  request_stream << "Cache-Control: max-age=0\r\n";
  request_stream << "Origin: http://localhost:8080\r\n";
  request_stream << "Upgrade-Insecure-Requests: 1\r\n";
  request_stream << "User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/55.0.2883.87 Safari/537.36\r\n";
  request_stream << "Content-Type: application/x-www-form-urlencoded\r\n";
  request_stream << "Accept: text/html,application/xhtml+xml,application/xml;q=0.9,image/webp,*/*;q=0.8\r\n";
  request_stream << "DNT: 1\r\n";
  request_stream << "Referer: http://localhost:8080/\r\n";
  request_stream << "Accept-Encoding: gzip, deflate, br\r\n";
  request_stream << "Accept-Language: de-DE,de;q=0.8,en-US;q=0.6,en;q=0.4\r\n";
  request_stream << "Cookie: org.cups.sid=772f03e1c42a7aff8df36a923bf4baf0; JSESSIONID.cdfec8e2=tcyr0fa105ar13hs51s60dbon; screenResolution=1920x1080; QT=1487290972991; SESSION=0510bf26-2b9c-46c1-ba02-f3b1a88ffc1c\r\n";
  request_stream << "\r\n";    
  request_stream << ss_to_send.str();
  request_stream << "\r\n";
  
  try
  {
    // Send the request.
    boost::asio::write(socket, request);
    
    boost::asio::streambuf response;
    boost::asio::read_until(socket, response, "\r\n");
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
}


/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool VelodyneDriver::poll(void)
{
  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  velodyne_msgs::VelodyneScanPtr scan(new velodyne_msgs::VelodyneScan);
  int n_packets = config_.npackets;
  
  if( config_.cut_angle >= 0) //Cut at specific angle feature enabled
  {
    scan->packets.reserve(n_packets);
    velodyne_msgs::VelodynePacket tmp_packet;
    while(true)
    {
      while(true)
      {
        int rc = input_->getPacket(&tmp_packet,  config_.time_offset);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
      }
      scan->packets.push_back(tmp_packet);
      
      static int last_base_rotation = -1;
      // Extract base rotation of first block in packet
      std::size_t rot_data_pos = 100*0+2;
      int base_rotation = *( (u_int16_t*) (&tmp_packet.data[rot_data_pos]));
      // Handle overflow 35999->0
      if(base_rotation<last_base_rotation)
        last_base_rotation-=36000;
      // Check if currently passing cut angle
      if(   last_base_rotation != -1
        && last_base_rotation < config_.cut_angle
        && base_rotation >= config_.cut_angle )
      {
        last_base_rotation = base_rotation;
        break; // Cut angle passed, one full revolution collected
      }
      last_base_rotation = base_rotation;
    }
  }
  else // standard behaviour
  {
    // Since the velodyne delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    scan->packets.resize(n_packets);
    for (int i = 0; i < n_packets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = input_->getPacket(&scan->packets[i],  config_.time_offset);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
      }
    }
  }
  
  // publish message using time of last packet read
  ROS_DEBUG("Publishing a full Velodyne scan.");
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  output_.publish(scan);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();

  return true;
}

void VelodyneDriver::callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  config_.time_offset = config.time_offset;
}

} // namespace velodyne_driver
