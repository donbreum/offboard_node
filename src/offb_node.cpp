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
#include <eigen3/Eigen/Eigen>

using namespace Eigen;
using namespace std;

void do_stuff(int* publish_rate, std::string &dd){
  //ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  //ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_b", 10);
  ros::Rate loop_rate(*publish_rate);
  while (ros::ok())
  {
      //dd++;
    //ROS_INFO_STREAM("Hello, from thread! Count: ");
    // std_msgs::Empty msg;
    // pub_b.publish(msg);
    loop_rate.sleep();
  }
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    TopicInformation topicinfo(&nh);
    cout << "cout msg "<< endl;
    // extra
    //string dad = 0;
    int rate_b = 5; // 1 Hz
    //boost::thread thread_b(do_stuff, &rate_b,boost::ref(dad));

    // own subscribe
    //

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO_STREAM("Hello, ROS!");

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1;
    pose.pose.position.y = 1;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // check how big error is and correct position
        // current position - target position
        //pose.pose.position.x = pose.pose.position.x + 0.1;
        cout << "x pos: " << pose.pose.position.x << endl;
        local_pos_pub.publish(pose);
        //ROS_INFO_STREAM("main thread - cnt: "+ dad);
        topicinfo.get_position_data();

        // cout << "Altitude local (own topic): " << alti.local << endl;
        // cout << "Altitude relative (own topic): " << alti.relative << endl;
        // cout << "Altitude terrain (own topic): " << alti.terrain << endl;
        // cout << "Altitude amsl (own topic): " << alti.amsl << endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
