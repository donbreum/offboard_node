#include "TopicInformation.hpp"


TopicInformation::TopicInformation(ros::NodeHandle* nodehandle)
 : nh_(*nodehandle){
  ROS_INFO_STREAM("TopicInformation constructor");
  initializeSubscribers();
}

void TopicInformation::initializeSubscribers(){
  ROS_INFO("Initializing subscribers in topic information");
  gps_pos_ = nh_.subscribe<sensor_msgs::NavSatFix>
    ("mavros/global_position/global", 10,&TopicInformation::nav_pos_cb,this);
  alt_pos_ = nh_.subscribe<mavros_msgs::Altitude>
    ("mavros/altitude", 10,&TopicInformation::altitude_cb,this);
}

void TopicInformation::get_position_data(){
  cout << "Latitude: " << nav_pos.latitude << endl;
  cout << "Longitude: " << nav_pos.longitude << endl;
  cout << "Altitude: " << nav_pos.altitude << endl;
}

void TopicInformation::altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
  altitude = *msg;
}

void TopicInformation::nav_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
  nav_pos = *msg;
}
