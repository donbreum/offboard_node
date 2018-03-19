#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <mavros_msgs/WaypointList.h> // mavros_msgs::WaypointList
#include "TopicInformation.hpp"
#include "tf/tf.h"
#include "angles/angles.h"
#include "pid.h"

class TopicInformation;
using namespace std;

class DroneControl{

public:

  DroneControl();
  DroneControl(TopicInformation tp);

  void update_drone_position();
  float distance_between_two_coords(float lat1, float lon1, float lat2,
                                    float lon2);
  float distance_between_two_coords_simple(float lat1, float lon1, float lat2,
                                    float lon2);
  float get_distance_to_current_waypoint();
  double get_bearing_to_current_waypoint();
  double get_bearing_to_current_waypoint_simple();
  float add_angles(float a1, float a2);
private:

  TopicInformation topic_message;
  std::vector<mavros_msgs::Waypoint> waypoint_list;
  TopicInformation tp;
  int current_waypoint_index = 0;
  double initial_heading_rad = M_PI/2;
  PID pid_height = PID(0.1, 0.5, -0.5, 1.0, 0.0, 0.0);
  //PID pid_heading = PID(0.1, 0.5, -0.5, 0.05, 0.05, 0.0001);
  PID pid_heading = PID(0.1, 0.5, -0.5, 1.0, 0.1, 0.01);
  bool is_north;
  bool is_east;

  int cnt = 0;
};


#endif
