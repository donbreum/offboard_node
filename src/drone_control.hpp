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

#include <numeric>

#define MAV_CMD_NAV_WAYPOINT 16
#define MAV_CMD_NAV_LOITER_UNLIM 17
#define MAV_CMD_NAV_LOITER_TURNS 18
#define MAV_CMD_NAV_LOITER_TIME 19
#define MAV_CMD_NAV_RETURN_TO_LAUNCH 20
#define MAV_CMD_NAV_LAND 21
#define MAV_CMD_NAV_TAKEOFF 22
#define MAV_CMD_NAV_LAND_LOCAL 23
#define MAV_CMD_NAV_TAKEOFF_LOCAL 24

// when a new waypoint is loaded, distance in meters
#define threshold_distance_to_waypoint 3

class TopicInformation;
using namespace std;

struct BodyVelocity{
  double linear_x;
  double linear_y;
  double linear_z;
  double angular_z;
};

class DroneControl{

public:

  DroneControl();
  DroneControl(TopicInformation tp);
  void update_drone_data();
  void update_drone_position();
  float distance_between_two_coords(float lat1, float lon1, float lat2,
                                    float lon2);
  float distance_between_two_coords_simple(float lat1, float lon1, float lat2,
                                    float lon2);
  float get_distance_to_current_waypoint();
  double get_bearing_to_current_waypoint();
  double get_bearing_to_current_waypoint_simple();
  float calculate_cross_track_error(float bearing_target, float distance_to_wp,
                                    float bearing_wp_to_wp);
  double get_bearing_between_two_waypoints(float current_lat, float current_lon,
                                           float target_lat, float target_lon);
  float add_angles(float a1, float a2);
  void calculate_velocity_body(float bearing, float heading, float height,
                               float bearing_pos_to_wp, float cross_track_err);
  void calculate_velocity_angular(float bearing, float heading);
  void calculate_velocity_vertical(float target_height, float height);
  void set_velocity_body();
  vector<double> get_target_heading_vector(float bearing);
  
private:

  TopicInformation tp;
  std::vector<mavros_msgs::Waypoint> waypoint_list;
  geometry_msgs::TwistStamped move_msg;
  int current_waypoint_index = 0;
  double initial_heading_rad = M_PI/2;
  BodyVelocity body_velocity;
  PID pid_height = PID(0.1, 0.5, -0.5, 1.0, 0.0, 0.0);
  //PID pid_heading = PID(0.1, 0.5, -0.5, 0.05, 0.05, 0.0001);
  PID pid_heading = PID(0.1, 0.5, -0.5, 1.0, 0.1, 0.01);
  PID pid_lat_cmd = PID(0.1, 3.0, -3.0, 0.3, 0.1, 0.001);
  bool is_north;
  bool is_east;

  int cnt = 0;

  bool takeoff_complete = false;

  float cur_pos_lat;
  float cur_pos_lon;
  float previous_wp_lat;
  float previous_wp_lon;
  float next_wp_lat;
  float next_wp_lon;
  float heading;
  float height;
};


#endif
