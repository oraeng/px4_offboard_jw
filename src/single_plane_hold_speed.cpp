/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include "jw_offboard_lib/controller.h"

geometry_msgs::PoseStamped PID_TEST;
void PID_TEST_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    PID_TEST = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_plane_hold_speed_Node");
    ros::NodeHandle nh;
    ros::Subscriber status_sub                  = nh.subscribe<mavros_msgs::State>("mavros/state", 10, status_cb);    
    ros::Subscriber curr_attitude_sub           = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,state_attitude_cb);
    ros::Subscriber speed_sub                   = nh.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud",100, state_speed_cb);
    ros::Publisher setpoint_attitude_pub        = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
    ros::ServiceClient arming_client            = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client          = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber jw_topic_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/jw_topic", 10, PID_TEST_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
////////////////////////////////////////////////////////////////////
// Check FCU connection and Offboard.
////////////////////////////////////////////////////////////////////
    // wait for FCU connection
    while(ros::ok() && !current_status.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

////////////////////////////////////////////////////////////////////
    

////////////////////////////////////////////////////////////////////
// Call class and def variables
////////////////////////////////////////////////////////////////////
    // Call Class object and init
    Altitude    altitude_obj(0.6, 0.0005, 0.001); // 0.6, 0.0005, 0.001일때 반응 좀 느린듯. (고도가 만힝 높아졌을떄?)
    Heading     heading_obj(1, 0, 0);
    Speed       speed_obj(0.6, 0.002, 0.0001);
    
    // def variables
    double quater_angle_cmd[4];
    double Current_attitude[3];
    double Desired_attitude[3];   // The desired angle we want to send
    double altitude;
    double thrust;
    double roll_cmd, pitch_cmd, yaw_cmd;
    bool Speed_Controller_Onoff = false;

    double Des_Alt = 50;
    double Des_Vel = 15;


    double UAV_pos[2];
    double psi;
    double Target_pos[2]    = {0,0};
    double R_ref            = 100;
////////////////////////////////////////////////////////////////////









    // 반복문 시작
    while(ros::ok()){
        if( current_status.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_status.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        quaternion_to_euler_angle(current_attitude, Current_attitude); // (타겟 메시지의 value/현재상태, 어떤 변수에 넣을건지)
        yaw_cmd             = 0;
        UAV_pos[0]          = current_attitude.pose.position.x;
        UAV_pos[1]          = current_attitude.pose.position.y;        

        roll_cmd            = Sidebearing_Controller(UAV_pos, Current_attitude[2], Target_pos, R_ref, current_speed.groundspeed);
        pitch_cmd           = -altitude_obj.Altitude_Controller(Des_Alt, current_attitude.pose.position.z);
        euler_angle_to_quaternion(roll_cmd, pitch_cmd, yaw_cmd, quater_angle_cmd);
        
        if (current_attitude.pose.position.z >= Des_Alt)
            Speed_Controller_Onoff = true;
        if (Speed_Controller_Onoff == true){
            // thrust          = speed_obj.Speed_Controller(Des_Vel, current_speed.airspeed);}
            thrust          = speed_obj.Speed_Controller(Des_Vel, current_speed.groundspeed);}
        else{
            thrust = 1;}


        
        
        // 메시지 퍼블리쉬.
        attitudecommand.orientation.w = quater_angle_cmd[0];
        attitudecommand.orientation.x = quater_angle_cmd[1];
        attitudecommand.orientation.y = quater_angle_cmd[2];
        attitudecommand.orientation.z = quater_angle_cmd[3];
        attitudecommand.thrust = thrust;
        setpoint_attitude_pub.publish(attitudecommand);


        ROS_INFO("================================");
        double rt = sqrt(pow(current_attitude.pose.position.x,2)+pow(current_attitude.pose.position.y,2));
        ROS_INFO("radius  : %f", rt);
        ROS_INFO("Yaw  : %f", Current_attitude);
        // ROS_INFO("roll  : %f", Current_attitude[0]*180/3.14);
        // // ROS_INFO("pitch_cmd  : %f", pitch_cmd);
        // // ROS_INFO("thrust  : %f", thrust);
        // ROS_INFO("Altitude  : %f", current_attitude.pose.position.z);
        // // ROS_INFO("{Speed} P : %f // I : %f // D : %f", P, I, D);
        // ROS_INFO("yaw_cmd : %f", yaw_cmd*180/3.14);
        // ROS_INFO("yaw   : %f", Current_attitude[2]*180/3.14);
        // ROS_INFO("UAV_vel: %f", current_speed.airspeed );
        // ROS_INFO("================================");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}