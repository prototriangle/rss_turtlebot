//
// Created by ignat on 01/11/2019.
//

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LidarFilter {
 public:

  ros::NodeHandle nh_;
  ros::Subscriber laser_sub;
  ros::Publisher laser_pub;

  bool debug;
  double filter_radius;
  std::string scan_in_topic;

  LidarFilter();
  void scanCallback(sensor_msgs::LaserScan);
};

LidarFilter::LidarFilter() : nh_("~") {

  nh_.param<bool>("debug", debug, false);
  if (debug) {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  nh_.param<std::string>("scan_in_topic", scan_in_topic, "/scan");
  nh_.param<double>("filter_radius", filter_radius, 0.1);

  // subscribers and publishers
  laser_sub = nh_.subscribe<sensor_msgs::LaserScan>(scan_in_topic, 1, &LidarFilter::scanCallback, this);
  laser_pub = nh_.advertise<sensor_msgs::LaserScan>("/scan_filtered", 1);
}

void LidarFilter::scanCallback(sensor_msgs::LaserScan scan_msg) {
  for (int i=0; i<scan_msg.ranges.size(); i++) {
    if (scan_msg.ranges[i] < scan_msg.range_min ||
        scan_msg.ranges[i] < filter_radius ||
        scan_msg.ranges[i] > scan_msg.range_max) {
      scan_msg.ranges[i] = 999;
      scan_msg.intensities[i] = 999;
    }
  }
  laser_pub.publish(scan_msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_filter");
  LidarFilter controller;
  ros::spin();
}
