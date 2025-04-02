#pragma once
#ifndef OctoArmTeleop_ROS2_OctoArmTeleop_NODE_HPP
#define OctoArmTeleop_ROS2_OctoArmTeleop_NODE_HPP

#include <cstdlib>
#include "adra/adra_api_serial.h"
#include "octo_arm_teleop_ros2/pid.hpp"

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
// Custom messages and services
#include "stm_client/msg/camarray.hpp"
#include "stm_client/msg/sendadrastatus.hpp"
#include "stm_client/srv/set_ik_mode.hpp"
#include "stm_client/srv/gu_iadrastat.hpp"
#include "stm_client/srv/relaycontrol.hpp"
#include "stm_client/srv/motorstatus.hpp"
#include <chrono>

enum JoyStick
{
    BASE_ROT_CW_CCW = 0,
    EE_HEIGHT_UP_DOWN = 1,
    EE_DEPTH_FW_BW = 3,
    EE_TOOL_ROT_CW_CCW = 4,
    EE_TOOL_NEXT_PREV = 5,
    CONVEX90 = 0,
    TOOL_ROT = 3,
    IDLE = 4,
    HORIZONTAL = 5,
    HOME = 6,
    WORK = 7,
};

enum InverseKinematicsModes
{
    POSITIVE_X = 0,
    NEGATIVE_X = 1
};

class OctoArmTeleop : public rclcpp::Node
{
public:
    OctoArmTeleop(const rclcpp::NodeOptions &options);
    ~OctoArmTeleop();
    void get_params();
    void init_subs_pubs_srvs();

private:
    std::string log_header_;
    bool debug_;
    bool init_teleop_;
    bool sim_;
    std::string com_port_;
    AdraApiSerial *adra_api_;
    int baud_rate_;
    int motors_;

   

    bool rot_base_cw_;
    bool rot_base_ccw_;
    bool ee_height_up_;
    bool ee_height_down_;
    bool ee_depth_fw_;
    bool ee_depth_bw_;
    bool ee_rot_cw_;
    bool ee_rot_ccw_;
    bool ee_tool_rot_cw_;
    bool ee_tool_rot_ccw_;
    bool ee_tool_next_;
    bool ee_tool_prev_;
    bool cng_tool_cmd_;
    bool rot_tool;
    bool work_config_flag_;
    bool home_config_flag_;
    bool horizontal_config_flag_;
    bool joy_work_flag_;
    bool joy_home_flag_;
    bool joy_horizontal_flag_;
    bool joy_convex90_flag_;
    bool error_flag_;
    bool auto_buff_down_;
    bool auto_buff_up_;
    int step;
    bool set_mode_;
    bool joy_priority_;
    int toggle_joy_flag_;
    double motor1_home_;
    double motor2_home_;
    double motor3_home_;
    double motor4_home_;
    double motor5_home_;
    double motor1_orientation_;
    double motor2_orientation_;
    double motor3_orientation_;
    double motor4_orientation_;
    double motor5_orientation_;
    double motor1_min_angle_;
    double motor2_min_angle_;
    double motor3_min_angle_;
    double motor4_min_angle_;
    double motor5_min_angle_;
    double motor1_max_angle_;
    double motor2_max_angle_;
    double motor3_max_angle_;
    double motor4_max_angle_;
    double motor5_max_angle_;
    double motor1_init_pos_;
    double motor2_init_pos_;
    double motor3_init_pos_;
    double motor4_init_pos_;
    double publish_rate_;
    double calc_rate_;
    float temp_drivers[4];
    float temp_motors[4];
    float volt_motors[4];
    uint8_t error_motors[4];
    bool brake_enabled_;
    bool intrupt_param_;
    bool auto_arm_control;
    double target_base_rotation_;
    double target_endeffector_height_;
    double target_endeffector_depth_;
    double target_endeffector_rotation_;
    double target_endeffector_tool_rotation_;
    double link1_length_;
    double link2_length_;
    double endeffector_x_;
    double endeffector_z_;
    double endeffector_head_th_;
    double relative_inclination_;
    int inverse_kinematics_mode_;
    sensor_msgs::msg::JointState target_joint_state_;
    stm_client::srv::Relaycontrol arm_;
    stm_client::msg::Sendadrastatus adra_status_;
    std_srvs::srv::Trigger::Request reset_force_;
    std_srvs::srv::SetBool::Request toggle_force_;
    std::string joy_topic_;
    std::string relative_inclination_topic_;
    std::string toggle_joy_topic_;
    std::string force_value_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr relative_inclination_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr force_value_sub_;
    std::string joint_state_topic_;
    std::string send_adra_status_topic_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<stm_client::msg::Sendadrastatus>::SharedPtr send_adra_status_pub_;
    std::string init_teleop_srv_name_;
    std::string stop_teleop_srv_name_;
    std::string set_ik_mode_srv_name_;
    std::string gui_adra_status_srv_name_;
    std::string reset_arm_srv_name_;
    std::string auto_buffing_srv_name_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_teleop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_teleop_srv_;
    rclcpp::Service<stm_client::srv::SetIkMode>::SharedPtr set_ik_mode_srv_;
    rclcpp::Service<stm_client::srv::GUIadrastat>::SharedPtr gui_adra_status_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_buffing_srv_;
    std::string toggle_relay_client_name_;
    std::string call_init_teleop_client_name_;
    std::string reset_force_feedback_name_;
    std::string toggle_force_feedback_name_;
    rclcpp::Client<stm_client::srv::Relaycontrol>::SharedPtr toggle_relay_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr call_init_teleop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_force_feedback_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr toggle_force_feedback_;
    void home_config();
    void work_config();
    void horizontal_config();
    void convex90_config();
    void publish_angles_(int i);
    void joy_callback_(const sensor_msgs::msg::Joy::SharedPtr msg);
    void toggle_joy_callback_(const std_msgs::msg::Bool::SharedPtr msg);
    void relative_inclination_callback_(const std_msgs::msg::Float64::SharedPtr msg);
    void force_callback_(const std_msgs::msg::Float32::SharedPtr msg);
    bool init_teleop_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    bool stop_teleop_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    bool set_ik_mode_srv_callback_(const std::shared_ptr<stm_client::srv::SetIkMode::Request> req,
                                   std::shared_ptr<stm_client::srv::SetIkMode::Response> res);
    bool gui_adra_status_srv_callback_(const std::shared_ptr<stm_client::srv::GUIadrastat::Request> req,
                                       std::shared_ptr<stm_client::srv::GUIadrastat::Response> res);
    bool reset_arm_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    bool auto_buffing_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void diagnose_motors();
    bool calculate_target_joint_state_();
    void next_tool_();
    void previous_tool_();
    void toggle_next_tool_();
    void toggle_previous_tool_();
    void rotate_base_cw_();
    void rotate_base_ccw_();
    void rotate_endeffector_tool_cw_();
    void rotate_endeffector_tool_ccw_();
    void increase_endeffector_height_();
    void decrease_endeffector_height_();
    void increase_endeffector_depth_();
    void decrease_endeffector_depth_();
    //void pid_tune_callback(OctoArmTeleop::PIDConfig &config, uint32_t level);
    double calculatePID(double _max, double _min, double _Kp, double _Ki, double _Kd, double _setpoint, double _pv);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr joy_state_reader_;
    rclcpp::TimerBase::SharedPtr teleop_timer_;
    rclcpp::TimerBase::SharedPtr pid_timer_;
    rclcpp::TimerBase::SharedPtr auto_buff_timer_;
    void timer_callback_();
    void teleop_timer_callback_();
    void joy_state_reader_callback_();
    void pid_callback_();
    void auto_buff_callback_();
    PID *zArm;
    double z_kp_;
    double z_ki_;
    double z_kd_;
    double min_;
    double max_;
    double force_req_;
    double pre_error_;
    double curr_force_;
    double prev_time_;
    int SampleTime;
    double _integral;
};

#endif
