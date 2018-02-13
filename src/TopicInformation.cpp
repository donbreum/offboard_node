#include "TopicInformation.hpp"


TopicInformation::TopicInformation(ros::NodeHandle* nodehandle) : nh_(*nodehandle){
  ROS_INFO_STREAM("TopicInformation constructor");
  initializeSubscribers();
  //ros::Subscriber nav_sb = nh->subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global", 10,nav_pos_cb);

}

void TopicInformation::initializeSubscribers(){
  ROS_INFO("Initializing subscribers in topic information");
  gps_pos_ = nh_.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global", 10,&TopicInformation::nav_pos_cb,this);
  alt_pos_ = nh.subscribe<mavros_msgs::Altitude> ("mavros/altitude", 10,altitude_cb);
}

string TopicInformation::get_position_data(){
  cout << "Latitude: " << nav_pos.latitude << endl;
  cout << "Longitude: " << nav_pos.longitude << endl;
  cout << "Altitude: " << nav_pos.altitude << endl;
  return number;
}

void altitude_cb(const mavros_msgs::Altitude::ConstPtr& msg){
  altitude = *msg;
}

void TopicInformation::nav_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
  nav_pos = *msg;
}
