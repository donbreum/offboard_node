#include "drone_control.hpp"

DroneControl::DroneControl(){
  cout << "constructor i dronecontrol "<< endl;
  waypoint_list = tp.get_waypoints();
  cout << "waypoints loaded in dronecontrol: " <<waypoint_list.size() << endl;
  if(waypoint_list.size() == 0)
    ROS_INFO("OBS!! NO WAYPOINTS LOADED");

  tp.initialize_pose();
  tp.set_mode();
  tp.arm(true);

  vector<float> position_data = tp.get_position_data();
  float heading = position_data[3];
  initial_heading_rad = angles::from_degrees(heading);
  cout << "initial heading: " << initial_heading_rad << endl;

}

DroneControl::DroneControl(TopicInformation tp){
  //topic_message = tp
}

vector<double> DroneControl::get_target_heading_vector(float bearing){
  double x_vec = add_angles(double(bearing), M_PI/2.0);
  double y_vec = add_angles(double(bearing), M_PI/2.0);
  x_vec = cos(x_vec);
  y_vec = sin(y_vec);

  if(y_vec < 0){
    if(is_north){
      y_vec = abs(y_vec);
    }
  }else
    if(!is_north){
      y_vec = y_vec * -1.0;
    }

  if(x_vec < 0){
    if(is_east)
      x_vec = abs(x_vec);
  }else{
      if(!is_east)
      x_vec = x_vec * -1.0;
    }

  vector<double> target_vector;
  target_vector.push_back(x_vec);
  target_vector.push_back(y_vec);
  return target_vector;
}

float DroneControl::distance_between_two_coords_simple(float lat1, float lon1,
                                                float lat2, float lon2){
  // OBS!! Does not work
  // float dlat_rad = (lat1 - lat2)*M_PI/180;
  // float dlon_rad = (lon1 - lon2)*M_PI/180;
  // float dlat = lat1 - lat2;
  // float dlon = lon1 - lon2;
  // float dist = sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5;
  // float dist_rad = sqrt((dlat_rad*dlat_rad) + (dlon_rad*dlon_rad)) * 1.113195e5;
  // cout << "dist:" << dist << endl;
  // cout << "dist_rad: " << dist_rad << endl;
  float dummy;
  return dummy;
}

float DroneControl::distance_between_two_coords(float lat1, float lon1,
                                                float lat2, float lon2){
  // return distance between two coordinates
  // might need to be simplified
  // https://www.movable-type.co.uk/scripts/latlong.html

  //cout << std::fixed << std::setprecision(9);

  float dlat_rad = (lat2 - lat1)*M_PI/180.0;
  float dlon_rad = (lon2 - lon1)*M_PI/180.0;
  // cout << "LONG DLON: " << (lon2 - lon1) << endl;
  // cout << "LONG DLAT: " << (lat2 - lat1) << endl;
  if( (lon2 - lon1) < 0){
    is_east = false;
    // cout << "vector is west" << endl;
  }
  else{
    is_east = true;
    // cout << "vector is pointing east" << endl;
  }
  if( (lat2 - lat1) < 0){
    is_north = false;
    // cout << "vector is pointing SOUTH" << endl;
  }
  else{
    is_north = true;
    // cout << "vector is pointing NORTH" << endl;
  }

  float lat1_rad = lat1*M_PI/180.0;
  float lat2_rad = lat2*M_PI/180.0;

  float a = sin(dlat_rad/2) * sin(dlat_rad/2) +
            cos(lat1_rad) * cos(lat2_rad) * sin(dlon_rad/2) * sin(dlon_rad/2);
  float c = 2 * atan2(sqrt(a),sqrt(1-a));
  float d = 6371e3 * c;  // distance in meters

  // cout << "Distance calculated between the coords: " << d << endl;

  return d; // return distance in meters
}

float DroneControl::get_distance_to_current_waypoint(){

  // return -1 if there is no more waypoints
  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  vector<float> position_data = tp.get_position_data();
  float current_lat = position_data[0];
  float current_long = position_data[1];
  float current_alt = position_data[3];

  float next_lat = waypoint_list[current_waypoint_index+1].x_lat;
  float next_long = waypoint_list[current_waypoint_index+1].y_long;
  float next_alt = waypoint_list[current_waypoint_index+1].z_alt;

  double r =  6371e3;
  double dist_lat = angles::to_degrees(next_lat - current_lat)/360.0;
  dist_lat = dist_lat * r;
  //cout << "dist lat: " << dist_lat/2 << endl;

  double dist_long = angles::to_degrees((next_long - current_long)/360);
  dist_long = dist_long * r;

  float distance_to_next =
      distance_between_two_coords(current_lat, current_long,
                                  next_lat, next_long);
  cout << "distance left: " << distance_to_next << endl;
  return distance_to_next;
}

float DroneControl::add_angles(float angle1, float angle2){

  float angle = angle1 + angle2;
  if(angle > 2.0*M_PI)
    angle -= M_PI;

  else if(angle < -0.0)
      angle += 2.0*M_PI;

  // cout << "added angle: " << angle << endl;
  return angle;
}

double DroneControl::get_bearing_to_current_waypoint_simple(){

  // vector<float> position_data = tp.get_position_data();
  // float current_lat = position_data[0];
  // float current_lon = position_data[1];
  // float target_lat = waypoint_list[current_waypoint_index].x_lat;
  // float target_lon = waypoint_list[current_waypoint_index].y_long;
  //
  // float off_x = target_lon - current_lon;
  // float off_y = target_lat - current_lat;
  // //float bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795;
  // float bearing = M_PI/2.0 + atan2(-off_y, off_x);// * 57.2957795;
  // //cout << "Bearing in simple method: " << bearing << endl;
  // if(bearing < 0)
  //     bearing += 2*M_PI;
      //bearing += 360.00;

  // cout << "Bearing in simple method: " << angles::to_degrees(bearing) << endl;
}
double DroneControl::get_bearing_to_current_waypoint(){

  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  vector<float> position_data = tp.get_position_data();
  float current_lat = position_data[0];
  float current_lon = position_data[1];
  cout << "pos to wp current: " << current_lat << " " << current_lon << endl;
  float target_lat = waypoint_list[current_waypoint_index].x_lat;
  float target_lon = waypoint_list[current_waypoint_index].y_long;
  cout << "pos to wp target: " << target_lat << " " << target_lon << endl;

  current_lat = current_lat * M_PI/180.0;
  target_lat = target_lat * M_PI/180.0;
  float dlon = (target_lon - current_lon) * M_PI/180.0;
  float y = sin(dlon) * cos(target_lat);
  float x = cos(current_lat) * sin(target_lat) -
            sin(current_lat) * cos(target_lat) * cos(dlon);
  float angle = atan2(y, x);
  float angle_rad = angle;

  angle_rad = angles::normalize_angle_positive(angle_rad);

  // double heading = position_data[3];
  angle = fmod(((angle * 180/M_PI) + 360), 360);
  cout << "Bearing in complex method: " << angle << endl;
  cout << "Bearing in complex method: " << angles::to_degrees(angle_rad) << endl;
  return angle_rad; // return in radians
  //return angle; // return angle in degrees
}

double DroneControl::get_bearing_between_two_waypoints(float current_lat,
                                                       float current_lon,
                                                       float target_lat,
                                                       float target_lon){
  cout << "current_waypoint_index" << current_waypoint_index << endl;

  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  // cout << "wp to wp current: " << current_lat << " " << current_lon << endl;
  // cout << "wp to wp target: " << target_lat << " " << target_lon << endl;

  current_lat = current_lat * M_PI/180.0;
  target_lat = target_lat * M_PI/180.0;
  float dlon = (target_lon - current_lon) * M_PI/180.0;
  float y = sin(dlon) * cos(target_lat);
  float x = cos(current_lat) * sin(target_lat) -
            sin(current_lat) * cos(target_lat) * cos(dlon);
  float angle = atan2(y, x);
  float angle_rad = angle;

  angle_rad = angles::normalize_angle_positive(angle_rad);

  // double heading = position_data[3];
  angle = fmod(((angle * 180.0/M_PI) + 360), 360);
  // cout << "Bearing in wp to wp: " << angle << endl;
  // cout << "Bearing in wp to wp: " << angles::to_degrees(angle_rad) << endl;
  return angle_rad; // return in radians
  //return angle; // return angle in degrees
}

void DroneControl::set_velocity_body(){
  vector<float> position_data = tp.get_position_data();
  float height = position_data[2];

  move_msg.twist.linear.z = body_velocity.linear_z;

  if(height > 2){
    // move_msg.twist.angular.z = z_angular_velocity;
    move_msg.twist.angular.z = body_velocity.angular_z;
    // move_msg.twist.linear.x = 5.0 * x_vec;
    // move_msg.twist.linear.y = 5.0 * y_vec;
    move_msg.twist.linear.x = body_velocity.linear_x;
    move_msg.twist.linear.y = body_velocity.linear_y;
    cout << "twist.linear.x: " << body_velocity.linear_x
     << " twist.linear.y: " << body_velocity.linear_y << endl;
  }

  tp.set_velocity(move_msg);
}

void DroneControl::calculate_velocity_body(float bearing, float heading,
                                           float height,
                                           float bearing_pos_to_wp,
                                           float cross_track_err){
  // cout << "heading: " << heading << endl;
  // cout << "bearing: " << bearing << endl;
  // cout << "bearing to wp from pos: " << bearing_pos_to_wp << endl;
  // cout << "bearing - bearing_pos_to_wp: " << bearing - bearing_pos_to_wp << endl;
  double shortest_dist = angles::shortest_angular_distance(
    heading, bearing);
  double shortest_dist_abs = abs(shortest_dist);
  double err = bearing - heading;

  if(shortest_dist < 0)
    err = -err;
  // cout << "shortest dist: " << shortest_dist << endl;
  double z_angular_velocity = pid_heading.calculate(0, shortest_dist);

  double heading_x = cos(add_angles(double(-heading), M_PI/2.0));
  double heading_y = sin(add_angles(double(-heading), M_PI/2.0));
  double bearing_to_wp_x = cos(add_angles(double(-bearing_pos_to_wp), M_PI/2.0));
  double bearing_to_wp_y = sin(add_angles(double(-bearing_pos_to_wp), M_PI/2.0));
  double vel_x = heading_x * 4.0; // should be multiplied with some gain
  double vel_y = heading_y * 4.0; // related to max speed

  double diff_bearing = bearing - bearing_pos_to_wp;
  // cout << "heading vector: " << vel_x <<" , " << vel_y << endl;
  double lateral_vel_x;
  double lateral_vel_y;
  if(diff_bearing < 0){
    // cout << "right perpendicular: " << heading_y << " , " <<  -heading_x << endl;
    lateral_vel_x = heading_y;
    lateral_vel_y = -heading_x;
  }
  else{
    // cout << "left perpendicular: " << -heading_y << " , " <<  heading_x << endl;
    lateral_vel_x = -heading_y;
    lateral_vel_y = heading_x;
  }
  // cout << "pre1: " << lateral_vel_x*bearing_to_wp_x << endl;
  // cout << "scalar product is: " << abs((lateral_vel_x*bearing_to_wp_x)+
  // (lateral_vel_y*bearing_to_wp_y)) << endl;
  double scalar = abs((lateral_vel_x*bearing_to_wp_x)+
                  (lateral_vel_y*bearing_to_wp_y));
  // if(scalar > 0.2)
  if(abs(shortest_dist) < (M_PI*0.05))
    scalar = scalar;
  else
    scalar = 0.0;

  double lateral_err = abs(pid_lat_cmd.calculate(0, cross_track_err));
  cout << "lateral_err" << lateral_err << endl;
  // cout << "scalar: " << scalar << endl;
  vel_x = vel_x + lateral_vel_x*lateral_err;
  vel_y = vel_y + lateral_vel_y*lateral_err;

  double vel_z = pid_height.calculate(5, height);
  body_velocity.linear_x = vel_x;
  body_velocity.linear_y = vel_y;
  body_velocity.linear_z = vel_z;
  body_velocity.angular_z = z_angular_velocity;

}

float DroneControl::calculate_cross_track_error(float bearing_target,
                                                float distance_to_wp,
                                                float bearing_wp_to_wp){

float theta = bearing_wp_to_wp - bearing_target;
float theta_deg = angles::from_degrees(theta);
if(theta_deg > 270)
  theta += 2.0*M_PI;
else if(theta_deg > 90 and theta_deg < 270)
  cout << "OBS OBS!! THERE IS SOME ERROR HERE" << endl;
theta = abs(theta);
// float theta1 = (M_PI/2.0)-theta;
float cross_track_error = distance_to_wp * sin(theta);
// float cross_track_error1 = distance_to_wp * cos(theta1);
cout << "cross_track_error: " << cross_track_error << endl;

return cross_track_error;
}

void DroneControl::update_drone_position(){
  float dist_to_next_wp = get_distance_to_current_waypoint();
  if(dist_to_next_wp < threshold_distance_to_waypoint){
    current_waypoint_index++;
  }

  update_drone_data();

  int MAV_CMD =  waypoint_list[current_waypoint_index].command;
  cout << "current command: " << MAV_CMD<< endl;
  cout << "current waypoint: " << current_waypoint_index << endl;
  if(takeoff_complete)
    cout << "takeoff complete " << endl;

  switch(MAV_CMD){
    case MAV_CMD_NAV_TAKEOFF:{
      float target_alt = waypoint_list[current_waypoint_index].z_alt; // target altitude
      body_velocity.linear_z =  pid_height.calculate(target_alt, height); //set height 0
      cout << "diff altitude: " << abs(target_alt - height) << endl;
      if(abs(target_alt - height) < 2){
        takeoff_complete = true;
        float bearing = get_bearing_between_two_waypoints(previous_wp_lat,
                                                          previous_wp_lon,
                                                          next_wp_lat,
                                                          next_wp_lon);
        float bearing_pos_to_wp = get_bearing_between_two_waypoints(cur_pos_lat,
                                                                    cur_pos_lon,
                                                                    next_wp_lat,
                                                                    next_wp_lon);

        float cross_track_error = calculate_cross_track_error(bearing_pos_to_wp,
                                                               dist_to_next_wp,
                                                               bearing);

      calculate_velocity_body(bearing, heading, height, bearing_pos_to_wp,
                              cross_track_error);
      }
      break;
    }
    case MAV_CMD_NAV_WAYPOINT:{
      float bearing = get_bearing_between_two_waypoints(previous_wp_lat,
                                                        previous_wp_lon,
                                                        next_wp_lat,
                                                        next_wp_lon);
      float bearing_pos_to_wp = get_bearing_between_two_waypoints(cur_pos_lat,
                                                                  cur_pos_lon,
                                                                  next_wp_lat,
                                                                  next_wp_lon);

      float cross_track_error = calculate_cross_track_error(bearing_pos_to_wp,
                                                             dist_to_next_wp,
                                                             bearing);

    calculate_velocity_body(bearing, heading, height, bearing_pos_to_wp,
                            cross_track_error);

      break;
    }
    // case MAV_CMD_NAV_LOITER_UNLIM:{
    //   body_velocity.linear_z =  pid_height.calculate(0, height); //set height 0
    //   break;
    // }
    // case MAV_CMD_NAV_LAND:{
    //
    //   break;
    // }
    default:{
      cout << "NO VALID COMMAND FROM MAVLINK - LAND AT LOCATION" << endl;
      body_velocity.linear_z = pid_height.calculate(0, height); // set height 0
      break;
      }
  }

  set_velocity_body();
}

void DroneControl::update_drone_data(){
  // these must be initaliezed in the constructor
  vector<float> position_data = tp.get_position_data();
  cur_pos_lat = position_data[0];
  cur_pos_lon = position_data[1];
  previous_wp_lat = waypoint_list[current_waypoint_index].x_lat;;
  previous_wp_lon = waypoint_list[current_waypoint_index].y_long;
  next_wp_lat = waypoint_list[current_waypoint_index+1].x_lat;
  next_wp_lon = waypoint_list[current_waypoint_index+1].y_long;
  heading = angles::from_degrees(position_data[3]);
  height = position_data[2];
}
