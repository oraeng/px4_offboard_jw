/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


geometry_msgs::PoseStamped current_position;
void curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

geometry_msgs::TwistStamped current_velocity;
void curr_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_velocity = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "const_alt_forward_vel_Node");
    ros::NodeHandle nh;

/*    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav0/mavros/set_mode");
*/

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav3/mavros/state", 10, state_cb);
    ros::Subscriber curr_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav3/mavros/local_position/pose", 10, curr_pos_cb);
    ros::Subscriber curr_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("uav3/mavros/local_position/velocity_body", 10, curr_vel_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav3/mavros/setpoint_position/local", 10);
    ros::Publisher body_rate_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("uav3/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav3/mavros/set_mode");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    // pose.pose.position.z = 2;

    geometry_msgs::TwistStamped bodyrate;
    bodyrate.twist.linear.x = 0.0;



    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        body_rate_pub.publish(bodyrate);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
//    offb_set_mode.request.custom_mode = "AUTO.LOITER";
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
    
        bodyrate.twist.linear.x = 1*(0-current_position.pose.position.x) + 0.1*(0-current_position.pose.position.x)*0.1 + 0.1*(0-current_position.pose.position.x)/0.1;
        bodyrate.twist.linear.y = 1*(0-current_position.pose.position.y) + 0.1*(0-current_position.pose.position.y)*0.1 + 0.1*(0-current_position.pose.position.y)/0.1;
        bodyrate.twist.linear.z = 1*(2-current_position.pose.position.z) + 0.1*(2-current_position.pose.position.z)*0.1 + 0.1*(2-current_position.pose.position.z)/0.1;
        // bodyrate.twist.linear.z = 1*(2-current_position.pose.position.z) + 0*(2-current_position.pose.position.z)*0.1 + 0*(2-current_position.pose.position.z)/0.1;


        // local_pos_pub.publish(pose);
        body_rate_pub.publish(bodyrate);
        // ROS_INFO("%f", current_position.z)
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}