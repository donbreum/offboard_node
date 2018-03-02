#include "drone_control.hpp"

DroneControl::DroneControl(){
}

DroneControl::DroneControl(TopicInformation tp){
  //topic_message = tp
}

void DroneControl::distance_between_two_coords(float lat, float lon){
    cout << "lat: " << lat << " long: " << lon << endl;
}

void DroneControl::update_drone_position(){
  geometry_msgs::PoseStamped pose;


  cnt++;
  cout << "cnt: " << cnt << endl;
  if(cnt > 300){
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
  }if(cnt < 300){
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

  }

    topic_message.pose(pose);

}
