/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/thread/thread.hpp>
#include <string>
#include "TopicInformation.hpp"
#include "drone_control.hpp"
#include <eigen3/Eigen/Eigen>
#include <mavros_msgs/WaypointList.h> // mavros_msgs::WaypointList
#include <mavros_msgs/Waypoint.h>


using namespace Eigen;
using namespace std;

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    //ros::NodeHandle nh;
    //TopicInformation topicinfo(&nh);
    TopicInformation topicinfo;
    DroneControl drone_control;


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO_STREAM("setup complete");


    topicinfo.initialize_pose();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    //
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    topicinfo.set_mode();
    topicinfo.arm(true);
    while(ros::ok()){

      drone_control.update_drone_position();
      cout << "waypoint index: " << topicinfo.get_waypoint_index() << endl;
      //topicinfo.pose(pose);

      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
