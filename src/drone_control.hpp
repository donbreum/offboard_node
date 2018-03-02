#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "TopicInformation.hpp"

class TopicInformation;
using namespace std;

class DroneControl{

public:

  DroneControl();
  DroneControl(TopicInformation tp);

  void update_drone_position();
  void distance_between_two_coords(float lat, float lon);

private:

  TopicInformation topic_message;

  int cnt = 0;
};


#endif
