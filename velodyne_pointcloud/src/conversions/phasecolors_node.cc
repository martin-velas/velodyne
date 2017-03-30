/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS node converts a Velodyne 3D LIDAR PointXYZIR cloud to
    PointXYZRGB, assigning colors for visualization of the laser
    rings.

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_msgs/FloatStamped.h>

/// @todo make sure these includes are really necessary
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace ros;
using namespace message_filters;
using namespace pcl;

ros::Publisher output_;

void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &inMsg) {
  ROS_INFO_STREAM("POINTS came at " << inMsg->header.stamp);
}
void phaseCallback(const velodyne_msgs::FloatStampedConstPtr &phaseMsg) {
  ROS_INFO_STREAM("PHASE came at " << phaseMsg->header.stamp);
}

void convertPoints(const sensor_msgs::PointCloud2ConstPtr &inMsg, const velodyne_msgs::FloatStampedConstPtr &phaseMsg)
{
  PointCloud<velodyne_pointcloud::PointXYZIR> in;
  fromROSMsg(*inMsg, in);

  // allocate an PointXYZRGB message with same time and frame ID as
  // input data
  PointCloud<PointXYZRGB>::Ptr outMsg(new PointCloud<PointXYZRGB>);
  outMsg->header.stamp = in.header.stamp;
  outMsg->header.frame_id = in.header.frame_id;
  outMsg->height = 1;

  for (size_t i = 0; i < in.size(); ++i)
    {
    PointXYZRGB p;
      p.x = in[i].x;
      p.y = in[i].y;
      p.z = in[i].z;

      double angle = atan2(p.x, -p.y) + M_PI;
      double phase_diff = angle - phaseMsg->val;
      if(phase_diff < 0.0) {
        phase_diff += 2*M_PI;
      }

      p.r = p.g = phase_diff/(2*M_PI) * 255;
      p.b = (1-phase_diff/2*M_PI) * 255;

      outMsg->push_back(p);
    }

  output_.publish(outMsg);
  ROS_DEBUG_STREAM("Published points: " << outMsg->size());
}

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "phase_colors_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  output_ = node.advertise<sensor_msgs::PointCloud2>("velodyne_phased_colored", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> points_input_(node, "velodyne_points", 1);
  message_filters::Subscriber<velodyne_msgs::FloatStamped> phase_input_(node, "velodyne_rotor_phase", 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, velodyne_msgs::FloatStamped> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), points_input_, phase_input_);
  sync.registerCallback(boost::bind(&convertPoints, _1, _2));

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
