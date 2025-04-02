#pragma once
#ifndef OCTO_ARM_TELEOP_ROS2_INDIVIDUAL_ARM_CONTROLLER_NODE_HPP
#define OCTO_ARM_TELEOP_ROS2_INDIVIDUAL_ARM_CONTROLLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64.hpp"
#include "adra/adra_api_serial.hpp"
#include <cstdlib>
#include <iostream>
#include <cstdio>
#include <vector>

enum JoyStick {
    M_PLUS_MINUS = 3,
    ACT_SELECT = 4,
    ACT_ENABLE = 10
};

class IndArmController : public rclcpp::Node {
public:
    IndArmController();
    ~IndArmController();
    void get_params();
    void init_subs_pubs_srvs();

private:
    std::string log_header_;
    bool debug_;
    bool init_teleop_;
    bool sim_;
    int motor_select_;
    double publish_rate_;
    double step_size_;
    std::string com_port_;
    int baud_rate_;
    AdraApiSerial *adra_api_;
    bool motor_select_toggle_;
    bool run_motor_;
    bool ee_m_pos_;
    bool ee_m_neg_;
    std::vector<bool> enabled_motors_;
    std::string joy_topic_;
    std::string joint_state_topic_;
    std::string init_teleop_srv_name_;
    std::string stop_teleop_srv_name_;


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_teleop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_teleop_srv_;

    sensor_msgs::msg::JointState target_joint_state_;
    
    double motor1_init_pos_;
    double motor2_init_pos_;
    double motor3_init_pos_;
    double motor4_init_pos_;
    double motor1_min_angle_;
    double motor2_min_angle_;
    double motor3_min_angle_;
    double motor4_min_angle_;
    double motor1_max_angle_;
    double motor2_max_angle_;
    double motor3_max_angle_;
    double motor4_max_angle_;


    rclcpp::TimerBase::SharedPtr joy_state_reader_;

    void joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr msg);
    bool init_teleop_srv_callback_(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    bool stop_teleop_srv_callback_(const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    void motor_m_positive_();
    void motor_m_negative_();
    void set_act_pos_();
    void get_init_pos_();
    void joy_state_reader_callback_(const rclcpp::TimerBase::SharedPtr);
};

#endif // OCTO_ARM_TELEOP_ROS2_INDIVIDUAL_ARM_CONTROLLER_NODE_HPP
