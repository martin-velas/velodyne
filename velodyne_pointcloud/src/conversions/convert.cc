/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <algorithm>

#include <velodyne_msgs/FloatStamped.h>
#include <velodyne_driver/input.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    rotor_phase_output_ =
        node.advertise<velodyne_msgs::FloatStamped>("velodyne_rotor_phase", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

    std::string output_timestamps_fn;
    private_nh.getParam("output_timestamps_file", output_timestamps_fn);
    if(!output_timestamps_fn.empty()) {
      output_timestamps.open(output_timestamps_fn.c_str());
    }
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    velodyne_msgs::FloatStamped rotor_phase;
    VPointCloud::Ptr outMsg(new VPointCloud());
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    rotor_phase.header.stamp = scanMsg->header.stamp;
    outMsg->header.frame_id = rotor_phase.header.frame_id = scanMsg->header.frame_id;
    outMsg->height = 1;

    rotor_phase.val = INFINITY;
    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      int last_size = outMsg->size();
      data_->unpack(scanMsg->packets[i], *outMsg);
      if(isinf(rotor_phase.val)) {
        for(VPointCloud::iterator pt = outMsg->begin(); pt < outMsg->end(); pt++) {
          rotor_phase.val = std::min(rotor_phase.val, atan2(pt->x, -pt->y)+M_PI);
        }
      }
      for(VPointCloud::iterator pt = outMsg->begin()+last_size; pt < outMsg->end(); pt++) {
        pt->phase = ((float) i) / scanMsg->packets.size();
      }
    }

    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp
                     << " phase: " << rotor_phase.val);
    rotor_phase_output_.publish(rotor_phase);
    output_.publish(outMsg);

    if(output_timestamps.is_open()) {
      output_timestamps << outMsg->header.stamp << " " << velodyne_driver::packet_time(scanMsg->packets[0]) << std::endl;
    }
  }

} // namespace velodyne_pointcloud
