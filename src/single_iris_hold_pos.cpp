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


double Pos_PID_Controller(double Global_Pos, double Des_Pos, double Max_vel){
    // 입력으로 Global Position 받음.
    // PID Controller
    double input = 0.5*(Des_Pos-Global_Pos) + 0.1*(Des_Pos-Global_Pos)*0.1 + 0.01*(Des_Pos-Global_Pos)/0.1;

    // Saturation
    if (input >= Max_vel){input = Max_vel;}
    else if (input <= -Max_vel){input = -Max_vel;}

    return input; //입력속도 출력함.
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_iris_hold_pos_Node");
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

    double Gazebo_Init_pos[2] = {10, 0};
    double Target_pos[2]    = {-5,5};
    double UAV_states[3]    = {current_position.pose.position.x, current_position.pose.position.y, 0};
    double max_vel    = 3;
    double psi        = 0;
    double Raidus     = 5;
    double Vel_uav    = 1;
    double Des_Alt    = 5;
    double Des_Pos[2]    = {10,0};
    double Position_Controll_Running = false;
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
        Des_Pos[0]    = des_Pos_msg.pose.position.x;
        Des_Pos[1]    = des_Pos_msg.pose.position.y;
        

        // Position 제어기
        if (current_position.pose.position.z >= Des_Alt-0.5) //고도가 원하는 고도보다 50cm 이상일 때, 위치제어기 실행
        {Position_Controll_Running = true;}
        
        if (Position_Controll_Running == true){
            // bodyrate.twist.linear.x = Pos_PID_Controller(UAV_states[0], Des_Pos[0],max_vel);
            // bodyrate.twist.linear.y = Pos_PID_Controller(UAV_states[1], Des_Pos[1],max_vel);}
            bodyrate.twist.angular.z = Pos_PID_Controller(psi , 0.78, max_vel); 
        }
        else {
            bodyrate.twist.linear.x = 0;
            bodyrate.twist.linear.y = 0;
        }

        bodyrate.twist.linear.z = Pos_PID_Controller(current_position.pose.position.z , Des_Alt, max_vel); 


        
        // 메시지 퍼블리쉬.
        body_rate_pub.publish(bodyrate);

        // LOGGER
        double rt = sqrt((Target_pos[0]-UAV_states[0])*(Target_pos[0]-UAV_states[0])+(Target_pos[1]-UAV_states[1])*(Target_pos[1]-UAV_states[1]));

        ROS_INFO("================================");
        // ROS_INFO("TEST_MSG: %f", des_Pos_msg.pose.position.x);
        ROS_INFO("Radius: %f", rt);
        ROS_INFO("UAV_x: %f", UAV_states[0] );
        ROS_INFO("UAV_y: %f", UAV_states[1]);
        ROS_INFO("Target_x: %f", Des_Pos[0]);
        ROS_INFO("Target_y: %f", Des_Pos[1]);
        ROS_INFO("psi: %f", psi);
        ROS_INFO("================================");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}