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
#include <cmath>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_position;
void curr_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}
geometry_msgs::PoseStamped des_Pos_msg;
void des_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    des_Pos_msg = *msg;
}
geometry_msgs::TwistStamped current_velocity;
void curr_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_velocity = *msg;
}
double quaternion_to_euler_angle(const geometry_msgs::PoseStamped msg){
    double  x = msg.pose.orientation.x;
	double  y = msg.pose.orientation.y;
	double  z = msg.pose.orientation.z;
	double  w = msg.pose.orientation.w;
        // ROS_INFO("x: %f", x)z
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


double Pos_PID_Controller(double States, double Des_States, double Max_vel, double Gain[3]){
    // 입력으로 Global Position 받음.
    // PID Controller
    static double error_prev;
    static double error_curr;
    static double I_term;
    double P_term;
    double D_term;
    double input;

    error_curr = Des_States-States;
    P_term = Gain[0]*error_curr;
    I_term = I_term + Gain[1]*error_curr;
    D_term = Gain[2]*(error_curr-error_prev); // dt is in the Gain.

    input = P_term + I_term + D_term;
    // 1 0.1 0.001
    // Saturation

    if (input >= Max_vel){input = Max_vel;}
    else if (input <= -Max_vel){input = -Max_vel;}

    error_prev = error_curr;
    return input; //입력속도 출력함.
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
    xt = Target_pos[0]+0.0001;
    yt = Target_pos[1]+0.0001;
    rt = sqrt((xt-x)*(xt-x)+(yt-y)*(yt-y));

    vt_x = 0;
    vt_y = 0;

    v_x = V_uav*cos(psi);
    v_y = V_uav*sin(psi);

    V_AT_Vec[0] = v_x-vt_x;
    V_AT_Vec[1] = v_y-vt_y;
    
    // e_n[0] = -sin(psi); // sin
    // e_n[1] = cos(psi);  // cos
    // e_TA[0] = (xt-x)/rt;
    // e_TA[1] = (yt-y)/rt;

    e_n[0] = -cos(psi); // sin
    e_n[1] = sin(psi);  // cos
    e_TA[0] = (yt-y)/rt;
    e_TA[1] = (xt-x)/rt;

    sin_eta = -(e_n[0]*e_TA[1] - e_TA[0]*e_n[1]);
    cos_csi = (v_x*V_AT_Vec[0] + v_y*V_AT_Vec[1])/sqrt(V_AT_Vec[0]*V_AT_Vec[0]+V_AT_Vec[1]*V_AT_Vec[1])/V_uav;
    // w = (V_uav*V_uav/R_ref) * (1+2*0.707*sin_eta)/cos_csi/V_uav;
    w = V_uav/R_ref + (5/V_uav)*sin_eta;
    ROS_INFO("eta: %f", asin(sin_eta)*180/3.14);
    // ROS_INFO("================================");
    // ROS_INFO("Radius: %f", rt);
    // ROS_INFO("UAV_x: %f", x );
    // ROS_INFO("UAV_y: %f", y);
    // ROS_INFO("Target_x: %f", xt);
    // ROS_INFO("Target_y: %f", yt);
    // ROS_INFO("================================");

    return -w;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_iris_sidebearing_Node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber curr_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, curr_pos_cb);
    // ros::Subscriber curr_glob_pos_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
    //         ("mavros/global_position/local", 10, curr_glob_pos_cb);
    ros::Subscriber curr_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_body", 10, curr_vel_cb);
    ros::Subscriber jw_topic_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/jw_topic", 10, des_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher body_rate_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
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

    // 필요한 변수 설정
    geometry_msgs::PoseStamped pose; // Publisher msg
    geometry_msgs::TwistStamped bodyrate;
    bodyrate.twist.linear.x = 0.0;
    bodyrate.twist.linear.y = 0.0;

    double Gazebo_Init_pos[2] = {0, 0};
    double Target_pos[2]    = {0,0};
    double UAV_states[3]    = {current_position.pose.position.x, current_position.pose.position.y, 0};
    double max_vel    = 20;
    double psi        = 0;
    double Raidus     = 3;
    double Vel_uav_ref    = 1;
    double Des_Alt    = 5;
    double Des_Pos[2]    = {0,0};
    double Position_Controll_Running = false;
    double Gain_x[3]   = {1.8, 0.0, 0};
    double Gain_y[3]   = {1.0, 0.0, 0};
    double Gain_alt[3] = {1.75, 0.01, 0.05}; // 1.5 Good
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        body_rate_pub.publish(bodyrate);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // 반복문 시작
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

        // 필요한 변수 입력
        psi = quaternion_to_euler_angle(current_position);
        UAV_states[0] = current_position.pose.position.x + Gazebo_Init_pos[0];
        UAV_states[1] = current_position.pose.position.y + Gazebo_Init_pos[1];
        UAV_states[2] = psi;
        // Des_Pos[0]    = des_Pos_msg.pose.position.x;
        // Des_Pos[1]    = des_Pos_msg.pose.position.y;
        

        // Position 제어기
        if (current_position.pose.position.z >= Des_Alt-0.5) //고도가원하는 고도보다 50cm 이상일 때, 위치제어기 실행
        {Position_Controll_Running = true;}
        

        


         

        if (Position_Controll_Running = true){

        bodyrate.twist.linear.x = Vel_uav_ref*cos(psi); //
        bodyrate.twist.linear.y = Vel_uav_ref*sin(psi); //
        bodyrate.twist.angular.z = Sidebearing_Controller(UAV_states, Des_Pos, Raidus, Vel_uav_ref);
        }
        
        
        bodyrate.twist.linear.z = Pos_PID_Controller(current_position.pose.position.z , Des_Alt, max_vel, Gain_alt); 


        
        // 메시지 퍼블리쉬.
        body_rate_pub.publish(bodyrate);

        // LOGGER
        double rt = sqrt((Target_pos[0]-UAV_states[0])*(Target_pos[0]-UAV_states[0])+(Target_pos[1]-UAV_states[1])*(Target_pos[1]-UAV_states[1]));

        ROS_INFO("================================");
        // ROS_INFO("TEST_MSG: %f", des_Pos_msg.pose.position.x);
        ROS_INFO("Radius: %f", rt);

        // ROS_INFO("x - P gain: %f // I gain: %f // D gain: %f //", Gain_x[0], Gain_x[1], Gain_x[1]);
        // ROS_INFO("y - P gain: %f // I gain: %f // D gain: %f //", Gain_y[0], Gain_y[1], Gain_y[2]);
        // ROS_INFO("z - P gain: %f // I gain: %f // D gain: %f //", Gain_alt[0], Gain_alt[1], Gain_alt[2]);

        ROS_INFO("Error: %f // Z_rate_cmd: %f", Des_Alt-current_position.pose.position.z, bodyrate.twist.linear.z);

        // ROS_INFO("UAV_[x,y]: [%f, %f], ", UAV_states[0], UAV_states[1]);
        // ROS_INFO("Target_[x,y]: [%f, %f]", Des_Pos[0], Des_Pos[1]);
        double V_ttt = sqrt(pow( bodyrate.twist.linear.x, 2)+ pow( bodyrate.twist.linear.y, 2));
        ROS_INFO("V_x : %f, V_y : %f, V_total : %f", bodyrate.twist.linear.x, bodyrate.twist.linear.y, V_ttt);
        ROS_INFO("yaw : %f", psi);

        ROS_INFO("yaw_rate_cmd : %f", bodyrate.twist.angular.z);
        ROS_INFO("linear_xy_rate_cmd : [%f, %f]", bodyrate.twist.linear.x, bodyrate.twist.linear.y);
        ROS_INFO("linear_xy_rate_cur : [%f, %f]", current_velocity.twist.linear.x, current_velocity.twist.linear.y);

        ROS_INFO("================================");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}