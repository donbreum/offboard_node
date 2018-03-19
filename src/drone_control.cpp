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



  // double val = 20;
  // for (int i = 0; i < 100; i++) {
  //     double inc = pid.calculate(0, val);
  //     printf("val:% 7.3f inc:% 7.3f\n", val, inc);
  //     val += inc;
  // }

}

DroneControl::DroneControl(TopicInformation tp){
  //topic_message = tp
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

  float dlat_rad = (lat2 - lat1)*M_PI/180;
  float dlon_rad = (lon2 - lon1)*M_PI/180;
  cout << "LONG DLON: " << (lon2 - lon1) << endl;
  cout << "LONG DLAT: " << (lat2 - lat1) << endl;
  if( (lon2 - lon1) < 0){
    is_east = false;
    cout << "vector is west" << endl;
  }
  else{
    is_east = true;
    cout << "vector is pointing east" << endl;
  }
  if( (lat2 - lat1) < 0){
    is_north = false;
    cout << "vector is pointing SOUTH" << endl;
  }
  else{
    is_north = true;
    cout << "vector is pointing NORTH" << endl;
  }



  float lat1_rad = lat1*M_PI/180;
  float lat2_rad = lat2*M_PI/180;

  float a = sin(dlat_rad/2) * sin(dlat_rad/2) +
            cos(lat1_rad) * cos(lat2_rad) * sin(dlon_rad/2) * sin(dlon_rad/2);
  float c = 2 * atan2(sqrt(a),sqrt(1-a));
  float d = 6371e3 * c;  // distance in meters

  cout << "Distance calculated between the coords: " << d << endl;

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

  float next_lat = waypoint_list[current_waypoint_index].x_lat;
  float next_long = waypoint_list[current_waypoint_index].y_long;
  float next_alt = waypoint_list[current_waypoint_index].z_alt;

  double r =  6371e3;
  double dist_lat = angles::to_degrees(next_lat - current_lat)/360.0;
  dist_lat = dist_lat * r;
  //cout << "dist lat: " << dist_lat/2 << endl;

  double dist_long = angles::to_degrees((next_long - current_long)/360);
  dist_long = dist_long * r;
  // cout << "dist long. " << dist_long/2 << endl;
  //
  // cout << "diff lat: " << (next_lat - current_lat) << endl;
  // cout << "diff lon: " << (next_long - current_long) << endl;

  float distance_to_next =
      distance_between_two_coords(current_lat, current_long,
                                  next_lat, next_long);

  return distance_to_next;
}

float DroneControl::add_angles(float angle1, float angle2){

  float angle = angle1 + angle2;
  if(angle > 2.0*M_PI)
    angle -= M_PI;

  else if(angle < -0.0)
      angle += 2.0*M_PI;

  cout << "added angle: " << angle << endl;
  return angle;
}

double DroneControl::get_bearing_to_current_waypoint_simple(){

  vector<float> position_data = tp.get_position_data();
  float current_lat = position_data[0];
  float current_lon = position_data[1];
  float target_lat = waypoint_list[current_waypoint_index].x_lat;
  float target_lon = waypoint_list[current_waypoint_index].y_long;

  float off_x = target_lon - current_lon;
  float off_y = target_lat - current_lat;
  //float bearing = 90.00 + atan2(-off_y, off_x) * 57.2957795;
  float bearing = M_PI/2 + atan2(-off_y, off_x);// * 57.2957795;
  //cout << "Bearing in simple method: " << bearing << endl;
  if(bearing < 0)
      bearing += 2*M_PI;
      //bearing += 360.00;

  cout << "Bearing in simple method: " << angles::to_degrees(bearing) << endl;
}
double DroneControl::get_bearing_to_current_waypoint(){

  if(current_waypoint_index > (waypoint_list.size()-1))
    return -1;

  vector<float> position_data = tp.get_position_data();
  float current_lat = position_data[0];
  float current_lon = position_data[1];
  float target_lat = waypoint_list[current_waypoint_index].x_lat;
  float target_lon = waypoint_list[current_waypoint_index].y_long;

  //var φ1 = this.lat.toRadians()
  current_lat = current_lat * M_PI/180;

  //var φ2 = point.lat.toRadians();
  target_lat = target_lat * M_PI/180;

  //var Δλ = (point.lon-this.lon).toRadians();
  float dlon = (target_lon - current_lon) * M_PI/180;

  //var y = Math.sin(Δλ) * Math.cos(φ2);
  float y = sin(dlon) * cos(target_lat);

  //var x = Math.cos(φ1)*Math.sin(φ2) - Math.sin(φ1)*Math.cos(φ2)*Math.cos(Δλ);
  float x = cos(current_lat) * sin(target_lat) - sin(current_lat) * cos(target_lat) * cos(dlon);

  //var θ = Math.atan2(y, x);
  float angle = atan2(y, x);

  //return (θ.toDegrees()+360) % 360;
  float angle_rad = angle;

  angle_rad = angles::normalize_angle_positive(angle_rad);

  // cout << "shortest distance: " << endl;
  double heading = position_data[3];
  // cout << "current heading in degrees: " << heading << endl;
  // cout << "current heading in radians: " << angles::from_degrees(heading)+initial_heading_rad << endl;
  angle = fmod(((angle * 180/M_PI) + 360), 360);
  // cout << "angle calc in deg after fmod: " << angle << endl;
  cout << "Bearing in complex method: " << angle << endl;
  return angle_rad; // return in radians
  //return angle; // return angle in degrees

}

void DroneControl::update_drone_position(){

  // cout << "waypoint index from drone control: " << tp.get_waypoint_index() << endl;
  // cout << "xlat: " << float(waypoint_list[tp.get_waypoint_index()].x_lat) << endl;
  // cout << "distance to next wp: " << get_distance_to_current_waypoint() << endl;
  // cout << "get bearing: " << get_bearing_to_current_waypoint() << endl;
  //current_waypoint_index = tp.get_waypoint_index();
  if(get_distance_to_current_waypoint() < 3){
    current_waypoint_index++;
  }
//  cout << "waypointindex drone control: " << current_waypoint_index << endl;

  //cout << add_angles(float(1.74532925),float(-5.23598776));
  cnt++;

  // here your desired angles
  get_bearing_to_current_waypoint_simple();
  vector<float> position_data = tp.get_position_data();
  float heading = angles::from_degrees(position_data[3]);
  float bearing = get_bearing_to_current_waypoint();
  cout << "TARGET::: " << bearing << endl;
  cout << "HEADING:::" << heading << endl;
  cout << "HEADING NORMAL: " << angles::normalize_angle(heading) << endl;
  double x_vec = add_angles(double(bearing), M_PI/2);
  double y_vec = add_angles(double(bearing), M_PI/2);
  cout << "add angles: x: " << x_vec << endl;
  cout << "add angles: y: " << y_vec << endl;
  cout << "target x: " << cos(x_vec) << endl;
  cout << "target y: " << sin(y_vec) << endl;
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
  // cout << "normalized bearing: " << angles::normalize_angle(bearing) << endl;
  // cout << "normalized headig: " << angles::normalize_angle(heading) << endl;
  // cout << "current heading in rad: " << angles::from_degrees(position_data[3]) << endl;
  // cout << "shortest dist in rad: " << angles::shortest_angular_distance(angles::from_degrees(position_data[3]), bearing) << endl;


  double shortest_dist = angles::shortest_angular_distance(angles::from_degrees(position_data[3]), bearing);
  cout << "shortest dist: " << shortest_dist << endl;
  //float difference = (angles::shortest_angular_distance(angles::from_degrees(position_data[3]), bearing));
  // cout << "Heading:::: " << angles::to_degrees(heading) << endl;
  // cout << "Heading Target::: " << angles::to_degrees(bearing) << endl;
  //
  // double shortest_dist = angles::shortest_angular_distance(heading, bearing);
  // cout << "angles shortest: " << angles::to_degrees(shortest_dist) << endl;
  // double final_heading = -angles::normalize_angle(bearing-intial_heading);
  //cout << "angles error: " << angles::to_degrees(shortest_dist-angles::from_degrees(heading)) << endl;
  //tf::Quaternion mav_orientation = tf::createQuaternionFromRPY( 0, 0, final_heading);

  geometry_msgs::TwistStamped move_msg;

  float height = position_data[2];

  double z_linear_velocity = pid_height.calculate(5, height);
  cout << "z_velocity: " << z_linear_velocity << endl;
  move_msg.twist.linear.z = z_linear_velocity;

  double shortest_dist_abs = abs(shortest_dist);
  cout << "shortest dist abs: " <<shortest_dist_abs << endl;
  double err = bearing - heading;
  cout << "error before correction: " << err << endl;
  if(shortest_dist < 0)
    err = -err;
  cout << "err: " << err << endl;
  double z_angular_velocity = pid_heading.calculate(0, shortest_dist);
  cout << "error angular: " << z_angular_velocity << endl;
  cout << "z_angluar_velocity: " << z_linear_velocity << endl;
  move_msg.twist.angular.z = z_angular_velocity;
  // if(err < 0){
  //   move_msg.twist.angular.z = -z_angular_velocity;
  //   cout << "minus " << endl;
  // }
  // else if (err >= 0){
  //   move_msg.twist.angular.z = z_angular_velocity;
  //   cout << "plus" << endl;
  // }

  if(height > 2){

      move_msg.twist.linear.x = 5.0 * x_vec;
      move_msg.twist.linear.y = 5.0 * y_vec;
      cout << "twist.linear.x: " << x_vec << " twist.linear.y: " << y_vec << endl;
  }

  geometry_msgs::PoseStamped pose;
  cout << "[CNT]: " << cnt << endl;

  topic_message.set_velocity(move_msg);
}
