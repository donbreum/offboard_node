#ifndef TOPICINFORMATION_H
#define TOPICINFORMATION_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

class TopicInformation{

public:
  TopicInformation(ros::NodeHandle* nodehandle);
  string get_position_data();
  void initializeSubscribers();

  void nav_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);

private:

ros::Subscriber gps_pos_;
ros::Subscriber alt_pos_;
ros::NodeHandle nh_;
sensor_msgs::NavSatFix nav_pos;
mavros_msgs::Altitude altitude;


string number = "tedn";
};


#endif
