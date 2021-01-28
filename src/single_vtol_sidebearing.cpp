/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


geometry_msgs::PoseStamped current_position;
void curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

// geometry_msgs::PoseWithCovarianceStamped current_glob_position;
// void curr_glob_pos_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
//     current_glob_position = *msg;
// }

geometry_msgs::TwistStamped current_velocity;
void curr_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_velocity = *msg;
}

double Sidebearing_Controller(double UAV_states[3], double Target_pos[2], double R_ref, double UAV_vel)
{   
    double x, y, psi, v_x, v_y, V_uav; 
    double xt, yt, vt_x, vt_y; 
    double rt;
    double V_AT_Vec[2], e_n[2], e_TA[2];
    double sin_eta, cos_csi, w;

    
    x  = UAV_states[0];
    y  = UAV_states[1];
    V_uav = UAV_vel;
    psi= UAV_states[2];
    // ROS_INFO("psi: %f", psi);
    xt = Target_pos[0];
    yt = Target_pos[1];
    rt = sqrt((xt-x)*(xt-x)+(yt-y)*(yt-y));

    vt_x = 0;
    vt_y = 0;

    v_x = V_uav*cos(psi);
    v_y = V_uav*sin(psi);

    V_AT_Vec[0] = v_x-vt_x;
    V_AT_Vec[1] = v_y-vt_y;
    
    e_n[0] = -sin(psi);
    e_n[1] = cos(psi);
    e_TA[0] = (xt-x)/rt;
    e_TA[1] = (yt-y)/rt;

    sin_eta = -(e_n[0]*e_TA[1] - e_TA[0]*e_n[1]);
    cos_csi = (v_x*V_AT_Vec[0] + v_y*V_AT_Vec[1])/sqrt(V_AT_Vec[0]*V_AT_Vec[0]+V_AT_Vec[1]*V_AT_Vec[1])/V_uav;
    
    w = (V_uav*V_uav/R_ref) * (1+2*0.707*sin_eta)/cos_csi/V_uav;
    // ROS_INFO("================================");
    // ROS_INFO("Radius: %f", rt);
    // ROS_INFO("UAV_x: %f", x );
    // ROS_INFO("UAV_y: %f", y);
    // ROS_INFO("Target_x: %f", xt);
    // ROS_INFO("Target_y: %f", yt);
    // ROS_INFO("================================");

    return -w;
}

double quaternion_to_euler_angle(double UAV_quart[4]){
    double	x = UAV_quart[0];
	double  y = UAV_quart[1];
	double  z = UAV_quart[2];
	double  w = UAV_quart[3];
    // ROS_INFO("x: %f", x);
    // ROS_INFO("y: %f", y);
    // ROS_INFO("z: %f", z);
    // ROS_INFO("w: %f", w);

    // // roll (x-axis rotation)
    // double sinr_cosp = 2 * (w * x + y * z);
    // double cosr_cosp = 1 - 2 * (x * x + y * y);
    // angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // // pitch (y-axis rotation)
    // double sinp = 2 * (w * y - z * x);
    // if (std::abs(sinp) >= 1)
    //     angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    // else
    //     angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    // ROS_INFO("yaw: %f", yaw*180/3.14);

    return yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_vtol_sidebearing_Node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber curr_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, curr_pos_cb);
    // ros::Subscriber curr_glob_pos_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
    //         ("mavros/global_position/local", 10, curr_glob_pos_cb);
    ros::Subscriber curr_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_body", 10, curr_vel_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher body_rate_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_raw/target_attitude", 10);
    ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Thrust>
            ("/mavros/setpoint_attitude/thrust", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 2;

    geometry_msgs::TwistStamped bodyrate;
    bodyrate.twist.linear.x = 0.0;

    geometry_msgs::Thrust thrust_power;
    thrust_power = 0.5

    double Gazebo_Init_pos[2] = {10, 0};
    double Target_pos[2]    = {-5,5};
    double UAV_states[3]    = {current_position.pose.position.x, current_position.pose.position.y, 0};
    double UAV_quart[4]     = {current_position.pose.orientation.x, current_position.pose.orientation.y, current_position.pose.orientation.z, current_position.pose.orientation.w};

    double psi        = 0;
    double Raidus     = 50;
    double Vel_uav    = 15;
    double Des_Alt    = 50;
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
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }



        UAV_quart[0]= current_position.pose.orientation.x;
        UAV_quart[1]= current_position.pose.orientation.y;
        UAV_quart[2]= current_position.pose.orientation.z;
        UAV_quart[3]= current_position.pose.orientation.w;
        psi = quaternion_to_euler_angle(UAV_quart);
        // ROS_INFO("psi: %f", psi);

        UAV_states[0] = current_position.pose.position.x + Gazebo_Init_pos[0];
        UAV_states[1] = current_position.pose.position.y + Gazebo_Init_pos[1];
        UAV_states[2] = psi;

        double rt = sqrt((Target_pos[0]-UAV_states[0])*(Target_pos[0]-UAV_states[0])+(Target_pos[1]-UAV_states[1])*(Target_pos[1]-UAV_states[1]));
        ROS_INFO("================================");
        ROS_INFO("Radius: %f", rt);
        ROS_INFO("UAV_x: %f", UAV_states[0] );
        ROS_INFO("UAV_y: %f", UAV_states[1]);
        ROS_INFO("Target_x: %f", Target_pos[0]);
        ROS_INFO("Target_y: %f", Target_pos[1]);
        ROS_INFO("================================");

        bodyrate.twist.angular.z = Sidebearing_Controller(UAV_states, Target_pos, Raidus, Vel_uav);
        bodyrate.twist.linear.x = Vel_uav; // -1
        bodyrate.twist.linear.z = 1*(Des_Alt-current_position.pose.position.z) + 0.1*(Des_Alt-current_position.pose.position.z)*0.1 + 0.1*(Des_Alt-current_position.pose.position.z)/0.1;


        // local_pos_pub.publish(pose);
        body_rate_pub.publish(bodyrate);
        thrust_pub.publish(thrust_power);
        // ROS_INFO("%f", current_position.z)
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}