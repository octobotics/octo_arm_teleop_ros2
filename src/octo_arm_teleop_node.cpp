#include "octo_arm_teleop_ros2/octo_arm_teleop_node.hpp"

OctoArmTeleop::OctoArmTeleop(const rclcpp::NodeOptions &options)
    : Node("octo_arm_teleop_node") 
{
    RCLCPP_INFO(this->get_logger(), "Node has been started");

    debug_ = false;
    sim_ = false;
    init_teleop_ = false;
    rot_base_cw_ = false;
    rot_base_ccw_ = false;
    ee_height_up_ = false;
    ee_height_down_ = false;
    ee_depth_fw_ = false;
    ee_depth_bw_ = false;
    ee_rot_cw_ = false;
    ee_rot_ccw_ = false;
    ee_tool_rot_cw_ = false;
    ee_tool_rot_ccw_ = false;
    ee_tool_next_ = false;
    ee_tool_prev_ = false;
    cng_tool_cmd_ = false;
    joy_priority_ = false;
    rot_tool = false;
    auto_arm_control = false;
    auto_buff_down_ = false;
    auto_buff_up_ = false;
    z_kp_ = 0.0;
    z_ki_ = 0.0;
    z_kd_ = 0.0;
    min_ = 0.4;
    max_ = 0.6;
    force_req_ = 0.5;
    curr_force_ = 0.0;
    _integral = 0.0;
    pre_error_ = 0.0;
    SampleTime = 1000;
    work_config_flag_ = false;
    home_config_flag_ = true;
    horizontal_config_flag_ = false;
    joy_work_flag_ = false;
    joy_home_flag_ = false;
    joy_horizontal_flag_ = false;
    motors_ = 4;
    step = 0;
    error_flag_ = false;
    log_header_ = "[OctoArmTeleop]: ";
    baud_rate_ = 926600;
    endeffector_head_th_ = 0.0;
    //com_port_ = "/dev/ttyUSB0";

    get_params(); 
    init_subs_pubs_srvs(); 

    target_joint_state_.name.resize(4);
    target_joint_state_.position.resize(4);
    target_joint_state_.velocity.resize(4);
    target_joint_state_.effort.resize(4);

    rclcpp::sleep_for(std::chrono::seconds(2)); // ROS 2 equivalent of sleep

    if (debug_)
        RCLCPP_INFO(this->get_logger(), "%sInitialized", log_header_.c_str());

     timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.5, 0),
      std::bind(&OctoArmTeleop::timer_callback_, this));

    teleop_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.5, 0),
      std::bind(&OctoArmTeleop::teleop_timer_callback_, this));

    joy_state_reader_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.5, 0),
      std::bind(&OctoArmTeleop::joy_state_reader_callback_, this));

    pid_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.5, 0),
      std::bind(&OctoArmTeleop::pid_callback_, this));


    auto_buff_timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      this->get_clock(),
      rclcpp::Duration(0.5, 0),
      std::bind(&OctoArmTeleop::auto_buff_callback_, this));
    
    //adra_api_ = new AdraApiSerial(com_port_.c_str(), baud_rate_);
}

OctoArmTeleop::~OctoArmTeleop()
{
    if (debug_)
        RCLCPP_INFO(this->get_logger(), "%sDestructor called", log_header_.c_str());
    delete adra_api_;
}

void OctoArmTeleop::get_params()
{
    com_port_ = this->declare_parameter<std::string>("/octo_arm_controller/port_name", "/dev/ttyUSB0");
    sim_ = this->declare_parameter<bool>("/octo_arm_teleop/sim", false);
    debug_ = this->declare_parameter<bool>("/octo_arm_teleop/debug", true);
    joint_state_topic_ = this->declare_parameter<std::string>("/octo_arm_teleop/joint_state_topic", "/octo_adra_ros/cmd_joint_states");
    joy_topic_ = this->declare_parameter<std::string>("/octo_arm_teleop/joy_topic", "/joy");
    publish_rate_ = this->declare_parameter<double>("/octo_arm_teleop/publish_rate", 300.0);
    calc_rate_ = this->declare_parameter<double>("/octo_arm_teleop/calc_rate", 400.0);
    motor1_home_ = this->declare_parameter<double>("/octo_arm_teleop/motor1/home_angle", 0.0);
    motor2_home_ = this->declare_parameter<double>("/octo_arm_teleop/motor2/home_angle", -0.0);
    motor3_home_ = this->declare_parameter<double>("/octo_arm_teleop/motor3/home_angle", 0.8);
    motor4_home_ = this->declare_parameter<double>("/octo_arm_teleop/motor4/home_angle", 0.0);
    link1_length_ = this->declare_parameter<double>("/octo_arm_teleop/link1_length", 0.28);
    link2_length_ = this->declare_parameter<double>("/octo_arm_teleop/link2_length", 0.275);
    motor1_orientation_ = this->declare_parameter<double>("/octo_arm_teleop/motor1/orientation", 1.0);
    motor2_orientation_ = this->declare_parameter<double>("/octo_arm_teleop/motor2/orientation", 1.0);
    motor3_orientation_ = this->declare_parameter<double>("/octo_arm_teleop/motor3/orientation", 1.0);
    motor4_orientation_ = this->declare_parameter<double>("/octo_arm_teleop/motor4/orientation", 1.0);
    motor1_min_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor1/min_angle", -1.75);
    motor2_min_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor2/min_angle", -3.141592);
    motor3_min_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor3/min_angle", -3.141592);
    motor4_min_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor4/min_angle", -6.28);
    motor1_max_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor1/max_angle", 1.75);
    motor2_max_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor2/max_angle", 3.141592);
    motor3_max_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor3/max_angle", 3.141592);
    motor4_max_angle_ = this->declare_parameter<double>("/octo_arm_teleop/motor4/max_angle", 6.28);
    relative_inclination_ = this->declare_parameter<double>("/octo_arm_teleop/relative_inclination", 0.0);
    inverse_kinematics_mode_ = this->declare_parameter<int>("/octo_arm_teleop/ik_mode", 0);
    baud_rate_ = this->declare_parameter<int>("/octo_arm_controller/baud_rate", 921600);

    // Check if the parameters were loaded correctly, else print warnings
    if (!this->has_parameter("/octo_arm_teleop/port_name"))
    {
        com_port_ = "/dev/ttyUSB0";
        RCLCPP_WARN(this->get_logger(), "Parameter port_name not found. Using default value: %s", com_port_.c_str());
    }
    if (!this->has_parameter("/octo_arm_teleop/sim"))
    {
        sim_ = false;
        RCLCPP_WARN(this->get_logger(), "Parameter sim not found. Using default value: %d", sim_);
    }
    if (!this->has_parameter("/octo_arm_teleop/debug"))
    {
        debug_ = true;
        RCLCPP_WARN(this->get_logger(), "Parameter debug not found. Using default value: %d", debug_);
    }
    if (!this->has_parameter("/octo_arm_teleop/publish_rate"))
    {
        publish_rate_ = 300.0;
        RCLCPP_WARN(this->get_logger(), "Parameter publish_rate not found. Using default value: %f", publish_rate_);
    }
    if (!this->has_parameter("/octo_arm_teleop/calc_rate"))
    {
        calc_rate_ = 400.0;
        RCLCPP_WARN(this->get_logger(), "Parameter calc_rate not found. Using default value: %f", calc_rate_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor1/home_angle"))
    {
        motor1_home_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "Parameter motor1_home_angle not found. Using default value: %f", motor1_home_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor2/home_angle"))
    {
        motor2_home_ = -0.0;
        RCLCPP_WARN(this->get_logger(), "Parameter motor2_home_angle not found. Using default value: %f", motor2_home_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor3/home_angle"))
    {
        motor3_home_ = 0.8;
        RCLCPP_WARN(this->get_logger(), "Parameter motor3_home_angle not found. Using default value: %f", motor3_home_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor4/home_angle"))
    {
        motor4_home_ = 0.0;
        RCLCPP_WARN(this->get_logger(), "Parameter motor4_home_angle not found. Using default value: %f", motor4_home_);
    }
    if (!this->has_parameter("/octo_adra_ros/cmd_joint_states"))
    {
        joint_state_topic_ = "/octo_adra_ros/cmd_joint_states";
        RCLCPP_WARN(this->get_logger(), "Parameter joint_state_topic not found. Using default value: %s", joint_state_topic_.c_str());
    }
    if (!this->has_parameter("/joy"))
    {
        joy_topic_ = "/joy";
        RCLCPP_WARN(this->get_logger(), "Parameter joy_topic not found. Using default value: %s", joy_topic_.c_str());
    }
    if (!this->has_parameter("/octo_arm_teleop/link1_length"))
    {
        link1_length_ = 0.28;
        RCLCPP_WARN(this->get_logger(), "Parameter link1_length not found. Using default value: %f", link1_length_);
    }
    if (!this->has_parameter("/octo_arm_teleop/link2_length"))
    {
        link2_length_ = 0.275;
        RCLCPP_WARN(this->get_logger(), "Parameter link2_length not found. Using default value: %f", link2_length_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor1/orientation"))
    {
        motor1_orientation_ = 1.0;
        RCLCPP_WARN(this->get_logger(), "Parameter motor1_orientation not found. Using default value: %f", motor1_orientation_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor2/orientation"))
    {
        motor2_orientation_ = 1.0;
        RCLCPP_WARN(this->get_logger(), "Parameter motor2_orientation not found. Using default value: %f", motor2_orientation_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor3/orientation"))
    {
        motor3_orientation_ = 1.0;
        RCLCPP_WARN(this->get_logger(), "Parameter motor3_orientation not found. Using default value: %f", motor3_orientation_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor4/orientation"))
    {
        motor4_orientation_ = 1.0;
        RCLCPP_WARN(this->get_logger(), "Parameter motor4_orientation not found. Using default value: %f", motor4_orientation_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor1/min_angle"))
    {
        motor1_max_angle_ = -1.75;
        RCLCPP_WARN(this->get_logger(), "Parameter motor1_max_angle not found. Using default value: %f", motor1_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor2/min_angle"))
    {
        motor2_max_angle_ = -3.141592;
        RCLCPP_WARN(this->get_logger(), "Parameter motor2_max_angle not found. Using default value: %f", motor2_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor3/min_angle"))
    {
        motor3_max_angle_ = -3.141592;
        RCLCPP_WARN(this->get_logger(), "Parameter motor3_max_angle not found. Using default value: %f", motor3_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor4/min_angle"))
    {
        motor4_max_angle_ = 6.28;
        RCLCPP_WARN(this->get_logger(), "Parameter motor4_max_angle not found. Using default value: %f", motor4_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor1/max_angle"))
    {
        motor1_max_angle_ = 1.75;
        RCLCPP_WARN(this->get_logger(), "Parameter motor1_max_angle not found. Using default value: %f", motor1_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor2/max_angle"))
    {
        motor2_max_angle_ = 3.141592;
        RCLCPP_WARN(this->get_logger(), "Parameter motor2_max_angle not found. Using default value: %f", motor2_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor3/max_angle"))
    {
        motor3_max_angle_ = 3.141592;
        RCLCPP_WARN(this->get_logger(), "Parameter motor3_max_angle not found. Using default value: %f", motor3_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/motor4/max_angle"))
    {
        motor4_max_angle_ = 6.28;
        RCLCPP_WARN(this->get_logger(), "Parameter motor4_max_angle not found. Using default value: %f", motor4_max_angle_);
    }
    if (!this->has_parameter("/octo_arm_teleop/relative_inclination"))
    {
        relative_inclination_ = 0.0;
       RCLCPP_WARN(this->get_logger(), "%sParameter relative_inclination not found. Using default value: %f", log_header_.c_str(),
                 relative_inclination_);
    }
    // if (!this->has_parameter("/octo_arm_teleop/ik_mode", inverse_kinematics_mode_))
    // {
    //     inverse_kinematics_mode_ = 0;
    //     RCLCPP_WARN(this->get_logger(), "%sParameter ik_mode not found. Using default value: %d", log_header_.c_str(),
    //              inverse_kinematics_mode_);
    // }
}

void OctoArmTeleop::init_subs_pubs_srvs()
{
    if (debug_)
        RCLCPP_INFO(this->get_logger(), "%sInitializing subscribers and publishers", log_header_.c_str());

    // Publishers
    send_adra_status_topic_ = "/send_arm_status";
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 100);
    send_adra_status_pub_ = this->create_publisher<stm_client::msg::Sendadrastatus>(send_adra_status_topic_, 1);

    // Subscribers
    relative_inclination_topic_ = "/octo_arm_teleop/relative_inclination";
    toggle_joy_topic_ = "/toggle_joy";
    force_value_topic_ = "/send_force_feedback";
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&OctoArmTeleop::joy_callback_, this, std::placeholders::_1));
    relative_inclination_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        relative_inclination_topic_, 1, std::bind(&OctoArmTeleop::relative_inclination_callback_, this, std::placeholders::_1));
    toggle_joy_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        toggle_joy_topic_, 1, std::bind(&OctoArmTeleop::toggle_joy_callback_, this, std::placeholders::_1));
    force_value_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        force_value_topic_, 1, std::bind(&OctoArmTeleop::force_callback_, this, std::placeholders::_1));

    // Service Servers
    init_teleop_srv_name_ = "/octo_arm_teleop/init_teleop";
    stop_teleop_srv_name_ = "/octo_arm_teleop/stop_teleop";
    set_ik_mode_srv_name_ = "/octo_arm_teleop/set_ik_mode";
    gui_adra_status_srv_name_ = "/gui_adra_status_srv";
    reset_arm_srv_name_ = "/octo_arm_teleop/reset_arm";
    auto_buffing_srv_name_ = "/octo_arm_teleop/auto_buff_pid";
    init_teleop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        init_teleop_srv_name_, std::bind(&OctoArmTeleop::init_teleop_srv_callback_, this, std::placeholders::_1, std::placeholders::_2));
    stop_teleop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        stop_teleop_srv_name_, std::bind(&OctoArmTeleop::stop_teleop_srv_callback_, this, std::placeholders::_1, std::placeholders::_2));
    set_ik_mode_srv_ = this->create_service<stm_client::srv::SetIkMode>(
        set_ik_mode_srv_name_, std::bind(&OctoArmTeleop::set_ik_mode_srv_callback_, this, std::placeholders::_1, std::placeholders::_2));
    gui_adra_status_srv_ = this->create_service<stm_client::srv::GUIadrastat>(
        gui_adra_status_srv_name_, std::bind(&OctoArmTeleop::gui_adra_status_srv_callback_, this, std::placeholders::_1, std::placeholders::_2));
    reset_arm_srv_ = this->create_service<std_srvs::srv::Trigger>(
        reset_arm_srv_name_, std::bind(&OctoArmTeleop::reset_arm_srv_callback_, this, std::placeholders::_1, std::placeholders::_2));
    auto_buffing_srv_ = this->create_service<std_srvs::srv::Trigger>(
        auto_buffing_srv_name_, std::bind(&OctoArmTeleop::auto_buffing_srv_callback_, this, std::placeholders::_1, std::placeholders::_2));

    // Service Clients
    toggle_relay_client_name_ = "/relay_toggle_channel";
    call_init_teleop_client_name_ = "/octo_arm_teleop/init_teleop";
    reset_force_feedback_name_ = "/reset_force_feedback_";
    toggle_force_feedback_name_ = "/toggle_force_feedback_";

    toggle_relay_client_ = this->create_client<stm_client::srv::Relaycontrol>(toggle_relay_client_name_);
    reset_force_feedback_ = this->create_client<std_srvs::srv::Trigger>(reset_force_feedback_name_);
    toggle_force_feedback_ = this->create_client<std_srvs::srv::SetBool>(toggle_force_feedback_name_);

    // Initialize variables
    auto request = std::make_shared<stm_client::srv::Relaycontrol::Request>();
    request->data = 8;
    auto result_future = toggle_relay_client_->async_send_request(request);
    toggle_joy_flag_ = -1;
    call_init_teleop_client_ = this->create_client<std_srvs::srv::Trigger>(call_init_teleop_client_name_);
}

void OctoArmTeleop::home_config()
{
    auto request = std::make_shared<stm_client::srv::Relaycontrol::Request>();
    request->data = 2;
    auto result_future = toggle_relay_client_->async_send_request(request);

    // ROS2 Rate at 20Hz
    rclcpp::Rate rate(20);
    int i;
    bool home[4] = {false};
    std::vector<double> home_posi = {-1.1000, 0.657094, 2.207022, -2.823289};

    if (init_teleop_)
    {
        for (i = 0; i < 4; i++)
        {
            while (!home[i])
            {
                RCLCPP_INFO(this->get_logger(), "target joint state is %f, %d", target_joint_state_.position[i], i);

                // Get the parameter in ROS2 using get_parameter()
                bool intrupt_param_ = this->get_parameter("intrupt_param").as_bool();

                if (intrupt_param_)
                {
                    if (abs(target_joint_state_.position[i] - home_posi[i]) > 0.02)
                    {
                        RCLCPP_INFO(this->get_logger(), "sub of target and home %f", target_joint_state_.position[i] - home_posi[i]);

                        if (home_posi[i] > target_joint_state_.position[i])
                        {
                            target_joint_state_.position[i] += 0.02;
                        }
                        else
                        {
                            target_joint_state_.position[i] -= 0.02;
                        }
                    }
                    else
                    {
                        home[i] = true;
                    }

                    publish_angles_(i);
                    rate.sleep();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "interrupted");
                    intrupt_param_ = false;
                    break;
                }
            }

            if (!intrupt_param_)
            {
                RCLCPP_WARN(this->get_logger(), "Idle button pressed");
                target_joint_state_.header.stamp = this->get_clock()->now();
                joint_state_pub_->publish(target_joint_state_);

                double m2_cur_pos, m3_cur_pos, m4_cur_pos;
                float pos;

                if (!sim_)
                {
                    // Actuator control logic with error checking
                    int ret = adra_api_->set_pos_target(i + 1, static_cast<float>(target_joint_state_.position[i]) * 100.0);
                    if (ret != 0 && debug_)
                    {
                        RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set target failed", log_header_.c_str(), i);
                    }

                    RCLCPP_INFO(this->get_logger(), "I am done");

                    // Get the current positions of actuators
                    int ret1 = adra_api_->get_pos_current(2, &pos);
                    if (ret1 != 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Error getting current position of actuator 2");
                        m2_cur_pos = -10.0;
                    }
                    else
                    {
                        m2_cur_pos = pos / 100.0;
                        RCLCPP_INFO(this->get_logger(), "Actuator 2 current position: %f", m2_cur_pos);
                    }

                    int ret2 = adra_api_->get_pos_current(3, &pos);
                    if (ret2 != 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Error getting current position of actuator 3");
                        m3_cur_pos = -10.0;
                    }
                    else
                    {
                        m3_cur_pos = pos / 100.0;
                        RCLCPP_INFO(this->get_logger(), "Actuator 3 current position: %f", m3_cur_pos);
                    }

                    int ret3 = adra_api_->get_pos_current(4, &pos);
                    if (ret3 != 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Error getting current position of actuator 4");
                        m4_cur_pos = -10.0;
                    }
                    else
                    {
                        m4_cur_pos = pos / 100.0;
                        RCLCPP_INFO(this->get_logger(), "Actuator 4 current position: %f", m4_cur_pos);
                    }

                    // Calculate end effector position
                    if (m2_cur_pos != -10.0 && m3_cur_pos != -10.0)
                    {
                        endeffector_z_ = link1_length_ * sin(m2_cur_pos) + link2_length_ * sin(m3_cur_pos + m2_cur_pos);
                        endeffector_x_ = link1_length_ * cos(m2_cur_pos) + link2_length_ * cos(m3_cur_pos + m2_cur_pos);
                        RCLCPP_INFO(this->get_logger(), "X:%f , Z:%f", endeffector_x_, endeffector_z_);
                    }
                }

                // Set the interrupt parameter in ROS2 using set_parameter()
                this->set_parameter(rclcpp::Parameter("intrupt_param", true));
                break;
            }
        }
    }
}

void OctoArmTeleop::work_config()
{
    auto request = std::make_shared<stm_client::srv::Relaycontrol::Request>();
    request->data = 2;
    auto result_future = toggle_relay_client_->async_send_request(request);

    rclcpp::Rate rate(20); // ROS2 Rate at 20Hz

    int i;
    bool work[4] = {false};
    std::vector<double> work_posi = {-0.8000, 1.27999, 1.7826, 0.091683}; // Work configuration positions

    // If teleop is initialized
    if (init_teleop_ == true)
    {
        for (i = 0; i < 4; i++)
        {
            while (work[i] == false)
            {
                RCLCPP_INFO(this->get_logger(), "Target joint state: %f, Joint: %d", target_joint_state_.position[i], i);

                // Get parameter value
                this->get_parameter("/intrupt_param", intrupt_param_);

                if (intrupt_param_)
                {
                    if (std::abs(target_joint_state_.position[i] - work_posi[i]) > 0.02)
                    {
                        RCLCPP_INFO(this->get_logger(), "Difference between target and work: %f", target_joint_state_.position[i] - work_posi[i]);

                        if (work_posi[i] > target_joint_state_.position[i])
                        {
                            target_joint_state_.position[i] += 0.02;
                        }
                        else
                        {
                            target_joint_state_.position[i] -= 0.02;
                        }
                    }
                    else
                    {
                        work[i] = true;
                    }

                    // Publish the joint angles
                    publish_angles_(i);
                    rate.sleep();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Interrupted");
                    break;
                }
            }

            if (!intrupt_param_)
            {
                RCLCPP_WARN(this->get_logger(), "Idle button pressed");

                // Publish joint states
                publish_angles_(0);
                publish_angles_(1);
                publish_angles_(2);
                publish_angles_(3);

                // Set parameter back to true
                this->set_parameter(rclcpp::Parameter("/intrupt_param", true));
                break;
            }
        }
    }
}

void OctoArmTeleop::horizontal_config()
{

    auto request = std::make_shared<stm_client::srv::Relaycontrol::Request>();
    request->data = 2;
    auto result_future = toggle_relay_client_->async_send_request(request);

    rclcpp::Rate rate(20); // ROS2 Rate at 20Hz

    int i;
    bool work[4] = {false};
    std::vector<double> horizontal_posi = {1.3060, 0.578669, 2.531715, -1.57}; // Horizontal configuration positions

    // If teleop is initialized
    if (init_teleop_ == true)
    {
        for (i = 0; i < 4; i++)
        {
            while (work[i] == false)
            {
                RCLCPP_INFO(this->get_logger(), "Target joint state: %f, Joint: %d", target_joint_state_.position[i], i);

                // Get parameter value
                this->get_parameter("/intrupt_param", intrupt_param_);

                if (intrupt_param_)
                {
                    if (std::abs(target_joint_state_.position[i] - horizontal_posi[i]) > 0.02)
                    {
                        RCLCPP_INFO(this->get_logger(), "Difference between target and horizontal: %f", target_joint_state_.position[i] - horizontal_posi[i]);

                        if (horizontal_posi[i] > target_joint_state_.position[i])
                        {
                            target_joint_state_.position[i] += 0.02;
                        }
                        else
                        {
                            target_joint_state_.position[i] -= 0.02;
                        }
                    }
                    else
                    {
                        work[i] = true;
                    }

                    // Publish the joint angles
                    publish_angles_(i);
                    rate.sleep();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Interrupted");
                    break;
                }
            }

            if (!intrupt_param_)
            {
                RCLCPP_WARN(this->get_logger(), "Idle button pressed");

                // Publish joint states
                publish_angles_(0);
                publish_angles_(1);
                publish_angles_(2);
                publish_angles_(3);

                // Set parameter back to true
                this->set_parameter(rclcpp::Parameter("/intrupt_param", true));
                break;
            }
        }
    }
}

void OctoArmTeleop::convex90_config()
{
    auto request = std::make_shared<stm_client::srv::Relaycontrol::Request>();
    request->data = 2;
    auto result_future = toggle_relay_client_->async_send_request(request);

    rclcpp::Rate rate(20); // ROS2 Rate at 20Hz

    int i;
    std::vector<double> convex90_posi = {-0.971518, 1.30, 1.722482, -1.71}; // Convex 90 configuration positions

    if (init_teleop_ == true)
    {
        for (i = 0; i < 4; i++)
        {
            while (true)
            {
                RCLCPP_INFO(this->get_logger(), "Target joint state: %f, Joint: %d", target_joint_state_.position[i], i);

                // Get parameter value
                this->get_parameter("/intrupt_param", intrupt_param_);

                if (intrupt_param_)
                {
                    if (std::abs(target_joint_state_.position[i] - convex90_posi[i]) > 0.02)
                    {
                        RCLCPP_INFO(this->get_logger(), "Difference between target and convex90: %f", target_joint_state_.position[i] - convex90_posi[i]);

                        if (convex90_posi[i] > target_joint_state_.position[i])
                        {
                            target_joint_state_.position[i] += 0.02;
                        }
                        else
                        {
                            target_joint_state_.position[i] -= 0.02;
                        }
                    }
                    else
                    {
                        break;
                    }

                    // Publish the joint angles
                    publish_angles_(i);
                    rate.sleep();
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Interrupted");
                    break;
                }
            }

            if (!intrupt_param_)
            {
                RCLCPP_WARN(this->get_logger(), "Idle button pressed");

                // Publish joint states
                publish_angles_(0);
                publish_angles_(1);
                publish_angles_(2);
                publish_angles_(3);

                // Set parameter back to true
                this->set_parameter(rclcpp::Parameter("/intrupt_param", true));
                break;
            }
        }
    }
}




void OctoArmTeleop::publish_angles_(int i)
{
   
    target_joint_state_.header.stamp = this->get_clock()->now();

    joint_state_pub_->publish(target_joint_state_);

    double m2_cur_pos;
    double m3_cur_pos;
    double m4_cur_pos;
    float pos;

    if (!sim_)
    {
        // Set the actuator position target
        int ret = adra_api_->set_pos_target(i + 1, static_cast<float>(target_joint_state_.position[i]) * 100.0);
        if (ret != 0)
        {
            if (debug_)
            {
                RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set target failed", log_header_.c_str(), i);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Actuator position target set successfully");

        // Get the current position of actuator 2
        int ret1 = adra_api_->get_pos_current(2, &pos);
        if (ret1 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting current position of actuator 2");
            m2_cur_pos = -10.0;
        }
        else
        {
            m2_cur_pos = pos / 100.0;
            RCLCPP_INFO(this->get_logger(), "Actuator 2 current position: %f", m2_cur_pos);
        }

        // Get the current position of actuator 3
        int ret2 = adra_api_->get_pos_current(3, &pos);
        if (ret2 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting current position of actuator 3");
            m3_cur_pos = -10.0;
        }
        else
        {
            m3_cur_pos = pos / 100.0;
            RCLCPP_INFO(this->get_logger(), "Actuator 3 current position: %f", m3_cur_pos);
        }

        // Get the current position of actuator 4
        int ret3 = adra_api_->get_pos_current(4, &pos);
        if (ret3 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting current position of actuator 4");
            m4_cur_pos = -10.0;
        }
        else
        {
            m4_cur_pos = pos / 100.0;
            RCLCPP_INFO(this->get_logger(), "Actuator 4 current position: %f", m4_cur_pos);
        }

        // Calculate the end effector position if the actuator values are valid
        if (m2_cur_pos != -10.0 && m3_cur_pos != -10.0)
        {
            endeffector_z_ = link1_length_ * sin(m2_cur_pos) + link2_length_ * sin(m3_cur_pos + m2_cur_pos);
            endeffector_x_ = link1_length_ * cos(m2_cur_pos) + link2_length_ * cos(m3_cur_pos + m2_cur_pos);
            endeffector_head_th_ = m2_cur_pos + m3_cur_pos + m4_cur_pos; // motor5_home_ + motor2_home_ + motor3_home_;
            RCLCPP_INFO(this->get_logger(), "X:%f , Z:%f", endeffector_x_, endeffector_z_);
        }
    }
}

void OctoArmTeleop::joy_callback_(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    if (init_teleop_)
    {
        if (joy->axes[BASE_ROT_CW_CCW] > 0.2)
        {
            rot_base_cw_ = true;
            rot_base_ccw_ = false;
        }
        else if (joy->axes[BASE_ROT_CW_CCW] < -0.2)
        {
            rot_base_ccw_ = true;
            rot_base_cw_ = false;
        }
        else
        {
            rot_base_cw_ = false;
            rot_base_ccw_ = false;
        }

        if (joy->axes[EE_HEIGHT_UP_DOWN] > 0.2)
        {
            ee_height_up_ = true;
            ee_height_down_ = false;
        }
        else if (joy->axes[EE_HEIGHT_UP_DOWN] < -0.2)
        {
            ee_height_down_ = true;
            ee_height_up_ = false;
        }
        else
        {
            ee_height_up_ = false;
            ee_height_down_ = false;
        }

        if (joy->axes[EE_DEPTH_FW_BW] > 0.2)
        {
            ee_depth_fw_ = true;
            ee_depth_bw_ = false;
        }
        else if (joy->axes[EE_DEPTH_FW_BW] < -0.2)
        {
            ee_depth_bw_ = true;
            ee_depth_fw_ = false;
        }
        else
        {
            ee_depth_fw_ = false;
            ee_depth_bw_ = false;
        }

        if (joy->axes[EE_TOOL_ROT_CW_CCW] == 1.0)
        {
            ee_tool_rot_cw_ = true;
            ee_tool_rot_ccw_ = false;
        }
        else if (joy->axes[EE_TOOL_ROT_CW_CCW] == -1.0)
        {
            ee_tool_rot_ccw_ = true;
            ee_tool_rot_cw_ = false;
        }
        else
        {
            ee_tool_rot_cw_ = false;
            ee_tool_rot_ccw_ = false;
        }

        if (joy->axes[EE_TOOL_NEXT_PREV] == 1.0)
        {
            ee_tool_next_ = true;
            ee_tool_prev_ = false;
            cng_tool_cmd_ = true;
        }
        else if (joy->axes[EE_TOOL_NEXT_PREV] == -1.0)
        {
            ee_tool_prev_ = true;
            ee_tool_next_ = false;
            cng_tool_cmd_ = true;
        }
        else
        {
            ee_tool_next_ = false;
            ee_tool_prev_ = false;
            cng_tool_cmd_ = false;
        }

        if (joy->buttons[TOOL_ROT])
        {
            rot_tool = !rot_tool;

            if (rot_tool)
            {
                ee_tool_next_ = true;
                ee_tool_prev_ = false;
                cng_tool_cmd_ = true;
                toggle_next_tool_();
                RCLCPP_INFO(this->get_logger(), "tool 1");
            }
            else
            {
                ee_tool_prev_ = true;
                ee_tool_next_ = false;
                cng_tool_cmd_ = true;
                toggle_previous_tool_();
                RCLCPP_INFO(this->get_logger(), "tool 2");
            }
        }

        if (joy->buttons[HOME] == 1)
        {
            joy_home_flag_ = !joy_home_flag_;
        }

        if (joy->buttons[WORK] == 1)
        {
            joy_work_flag_ = !joy_work_flag_;
        }

        if (joy->buttons[HORIZONTAL] == 1)
        {
            joy_horizontal_flag_ = !joy_horizontal_flag_;
        }

        if (joy->buttons[CONVEX90])
        {
            joy_convex90_flag_ = !joy_convex90_flag_;
        }

        if (joy->buttons[9])
        {
            auto result = reset_force_feedback_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }
    }
}

void OctoArmTeleop::next_tool_()
{
    /**
     * @brief Switch to the next tool
     * @param None
     * @return None
     */
    if (init_teleop_)
    {
        if (cng_tool_cmd_)
        {
            endeffector_head_th_ += motor4_orientation_ * 1.57079632679;
            target_joint_state_.position[3] += motor4_orientation_ * 1.57079632679;
            cng_tool_cmd_ = false;
            ee_tool_next_ = false;
        }
    }
}

void OctoArmTeleop::previous_tool_()
{
    /**
     * @brief Switch to the previous tool
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        if (cng_tool_cmd_)
        {
            endeffector_head_th_ -= motor4_orientation_ * 1.57079632679;
            target_joint_state_.position[3] -= motor4_orientation_ * 1.57079632679;
            cng_tool_cmd_ = false;
            ee_tool_prev_ = false;
        }
    }
}

void OctoArmTeleop::toggle_joy_callback_(const std_msgs::msg::Bool::SharedPtr msg)
{

    toggle_joy_flag_ = msg->data;
}

void OctoArmTeleop::relative_inclination_callback_(const std_msgs::msg::Float64::SharedPtr msg)
{
    /**
     * @brief Callback for relative inclination topic
     * @param msg: relative inclination
     */
    RCLCPP_INFO(this->get_logger(), "%sRelative inclination received: %f", log_header_.c_str(), msg->data);
    relative_inclination_ = msg->data;
}

void OctoArmTeleop::force_callback_(const std_msgs::msg::Float32::SharedPtr msg)
{
    curr_force_ = abs(msg->data);
    // ROS_INFO("%sforce:%f",log_header_.c_str(),curr_force_);
}

bool OctoArmTeleop::init_teleop_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    /**
     * @brief Initialize the teleoperation service callback
     * @param req - service request
     * @tyoe std_srvs::Trigger::Request
     * @param res - service response
     * @tyoe std_srvs::Trigger::Response
     * @return true if the teleoperation was successfully initialized
     * @details This service callback initializes the teleoperation.
     */

    RCLCPP_INFO(this->get_logger(), "%sInitializing teleop", log_header_.c_str());
    //sim_ = this->declare_parameter<bool>("/octo_arm_teleop/sim", false);
    this->get_parameter("/octo_arm_teleop/sim", sim_);
    adra_api_ = new AdraApiSerial(com_port_.c_str(), baud_rate_);
    // sim_ = true;
    //    ROS_INFO("shutting down");
    if (sim_ == false)

    {
        //adra_api_ = new AdraApiSerial(com_port_.c_str(), baud_rate_);

        /// Set actuator mode to POSITION
        /// actuator 1
        int setPOS = adra_api_->into_motion_mode_pos(1);
        if (setPOS == 0)
        {
            RCLCPP_INFO(this->get_logger(), "%s Actuator %d set to POSITION", log_header_.c_str(), 1);
        }
        else
        {
            error_flag_ = true;
            RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set to POSITION failed", log_header_.c_str(), 1);
        }

        /// actuator 2
        setPOS = adra_api_->into_motion_mode_pos(2);
        if (setPOS == 0)
        {
            RCLCPP_INFO(this->get_logger(), "%s Actuator %d set to POSITION", log_header_.c_str(), 2);
        }
        else
        {
            if (error_flag_ == false)
            {
                error_flag_ = true;
               RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set to POSITION failed", log_header_.c_str(), 2);
            }
        }

        /// actuator 3
        setPOS = adra_api_->into_motion_mode_pos(3);
        if (setPOS == 0)
        {
            RCLCPP_INFO(this->get_logger(), "%s Actuator %d set to POSITION", log_header_.c_str(), 3);
        }
        else
        {

            if (error_flag_ == false)
            {
                error_flag_ = true;
                RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set to POSITION failed", log_header_.c_str(), 2);
            }
        }

        /// actuator 4
        setPOS = adra_api_->into_motion_mode_pos(4);
        if (setPOS == 0)
        {
            RCLCPP_INFO(this->get_logger(), "%s Actuator %d set to POSITION", log_header_.c_str(), 4);
        }
        else
        {

            if (error_flag_ == false)
            {
                error_flag_ = true;
                RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set to POSITION failed", log_header_.c_str(), 2);
            }
        }

        if (error_flag_ == true)
        {
            init_teleop_ = true;
            res->success = false;
            res->message = "Check error ";
            return true;
        }
        else
        {
            /// Enable 5 actuators
            int resp = adra_api_->into_motion_enable(1);
            if (resp == 0)
                RCLCPP_INFO(this->get_logger(), "%s ActuatorEmpty %d enabled", log_header_.c_str(), 1);
            else
                RCLCPP_INFO(this->get_logger(), "%s Actuator %d not enabled", log_header_.c_str(), 1);
            resp = adra_api_->into_motion_enable(2);
            if (resp == 0)
                RCLCPP_INFO(this->get_logger(), "%s Actuator %d enabled", log_header_.c_str(), 2);
            else
                RCLCPP_INFO(this->get_logger(), "%s Actuator %d not enabled", log_header_.c_str(), 2);
            resp = adra_api_->into_motion_enable(3);
            if (resp == 0)
                RCLCPP_INFO(this->get_logger(), "%s Actuator %d enabled", log_header_.c_str(), 3);
            else
                RCLCPP_INFO(this->get_logger(), "%s Actuator %d not enabled", log_header_.c_str(), 3);

            resp = adra_api_->into_motion_enable(4);
            if (resp == 0)
                RCLCPP_INFO(this->get_logger(), "%s Actuator %d enabled", log_header_.c_str(), 4);
            else
                RCLCPP_INFO(this->get_logger(), "%s Actuator %d not enabled", log_header_.c_str(), 4);

            rclcpp::sleep_for(std::chrono::milliseconds(2000));

            float pos;
            int ret;

            ret = adra_api_->get_pos_current(1, &pos);

            if (ret != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "%sError getting current position of actuator 1", log_header_.c_str());
                throw std::runtime_error("Error getting current position");
            }
            else
            {
               RCLCPP_INFO(this->get_logger(), "%sActuator 1 current position: %f", log_header_.c_str(), pos / 100.0);
                motor1_init_pos_ = pos / 100.0;
            }

            ret = adra_api_->get_pos_current(2, &pos);
            if (ret != 0)
            {
                RCLCPP_INFO(this->get_logger(), "%sError getting current position of actuator 2", log_header_.c_str());
                throw std::runtime_error("Error getting current position");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "%sActuator 2 current position: %f", log_header_.c_str(), pos / 100.0);
                motor2_init_pos_ = pos / 100.0;
            }

            ret = adra_api_->get_pos_current(3, &pos);
            if (ret != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "%sError getting current position of actuator 3", log_header_.c_str());
                throw std::runtime_error("Error getting current position");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "%sActuator 3 current position: %f", log_header_.c_str(), pos / 100.0);
                motor3_init_pos_ = pos / 100.0;
            }

            ret = adra_api_->get_pos_current(4, &pos);
            if (ret != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "%sError getting current position of actuator 4", log_header_.c_str());
                throw std::runtime_error("Error getting current position");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "%sActuator 4 current position: %f", log_header_.c_str(), pos / 100.0);
                motor4_init_pos_ = pos / 100.0;
            }

            target_joint_state_.position[0] = motor1_init_pos_;
            target_joint_state_.position[1] = motor2_init_pos_;
            target_joint_state_.position[2] = motor3_init_pos_;
            target_joint_state_.position[3] = motor4_init_pos_;
            endeffector_head_th_ = motor2_init_pos_ + motor3_init_pos_ + motor4_init_pos_; // motor5_home_ + motor2_home_ + motor3_home_;

            endeffector_z_ = link1_length_ * sin(motor2_init_pos_) + link2_length_ * sin(motor3_init_pos_ + motor2_init_pos_);
            endeffector_x_ = link1_length_ * cos(motor2_init_pos_) + link2_length_ * cos(motor3_init_pos_ + motor2_init_pos_);
            RCLCPP_INFO(this->get_logger(), "X:%f , Z:%f", endeffector_x_, endeffector_z_);

            init_teleop_ = true;
            res->success = true;
            res->message = "Teleop initialized";
            return true;
        }
    }
    res->message = "Inside simulation";
    return false;
}

bool OctoArmTeleop::stop_teleop_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    /**
     * @brief Stop the teleoperation service callback
     * @param req - service request
     * @tyoe std_sdiagnose_motorsrvs::Trigger::Request
     * @param res - service response
     * @tyoe std_srvs::Trigger::Response
     * @return true if the teleoperation was successfully stopped
     * @details This service callback stops the teleoperation.
     */

    RCLCPP_INFO(this->get_logger(), "%sStopping teleop", log_header_.c_str());
    if (!sim_)
    {
        if (init_teleop_)
        {
            int ret;

            for (int i = 1; i <= 4; i++)
            {

                ret = adra_api_->into_motion_disable(i);
                if (ret != 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "%sError disabling actuator", log_header_.c_str());
                    return false;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "%sActuator %d disabled", log_header_.c_str(), i);
                }
            }
            // delete adra_api_;
            init_teleop_ = false;
            res->success = true;
            res->message = "Teleop stopped";
            return true;
        }
        else
        {
            init_teleop_ = false;
            res->success = true;
            res->message = "Teleop not initialized";
            return true;
        }
    }
    delete adra_api_;
    res->message = "Inside simulation";
    return false;
}

bool OctoArmTeleop::set_ik_mode_srv_callback_(const std::shared_ptr<stm_client::srv::SetIkMode::Request> req,
                                   std::shared_ptr<stm_client::srv::SetIkMode::Response> res)
{
    /**
     * @brief Set IK mode service callback
     * @param req - service request
     * @tyoe octo_arm_teleop::SetIKMode::Request
     * @param res - service response
     * @tyoe octo_arm_teleop::SetIKMode::Response
     * @return true if the IK mode was successfully set
     * @details This service callback sets the IK mode.
     */

    RCLCPP_INFO(this->get_logger(), "%sSetting IK mode to %d", log_header_.c_str(), (int)req->ik_mode_id);
    inverse_kinematics_mode_ = req->ik_mode_id;
    res->success = true;
    res->message = "IK mode set";
    return true;
}

bool OctoArmTeleop::gui_adra_status_srv_callback_(const std::shared_ptr<stm_client::srv::GUIadrastat::Request> req,
                                       std::shared_ptr<stm_client::srv::GUIadrastat::Response> res)
{
    /**
     * @brief GUI_adra_status service callback
     * @param req - service request
     * @tyoe octo_arm_teleop::GUI_adra_stat::Request
     * @param res - service response
     * @tyoe octo_arm_teleop::GUI_adra_stat::Response
     *
     */
    for (int id = 1; id < 5; id++)
    {
        adra_api_->get_temp_driver(id, &temp_drivers[id - 1]);
        adra_api_->get_temp_motor(id, &temp_motors[id - 1]);
        adra_api_->get_bus_volt(id, &volt_motors[id - 1]);
        adra_api_->get_error_code(id, &error_motors[id - 1]);

        res->temp_drivers[id - 1] = temp_drivers[id - 1];
        res->temp_motors[id - 1] = temp_motors[id - 1];
        res->volt_motors[id - 1] = volt_motors[id - 1];
        res->error_motors[id - 1] = error_motors[id - 1];
    }
    return true;
}

bool OctoArmTeleop::reset_arm_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    diagnose_motors();
    return true;
}

/// helper functions
// Diagnostic function
void OctoArmTeleop::diagnose_motors()
{
    if (step == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Step 0");
        rclcpp::sleep_for(std::chrono::milliseconds(3000));
        auto request = std::make_shared<stm_client::srv::Relaycontrol::Request>();
        request->data = 8;
        auto result_future = toggle_relay_client_->async_send_request(request);
        // sleep for 1 second
        RCLCPP_INFO(this->get_logger(), "arm off");
        rclcpp::sleep_for(std::chrono::milliseconds(5000));
        result_future = toggle_relay_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "arm on");
        rclcpp::sleep_for(std::chrono::milliseconds(3000));

        step = 2;
        RCLCPP_INFO(this->get_logger(), "%sNode killed", log_header_.c_str());
        init_teleop_ = false;

        delete adra_api_;
        //delete timer_;
        //delete joy_state_reader_;
        rclcpp::shutdown();
        RCLCPP_INFO(this->get_logger(), "%sNode killed", log_header_.c_str());
    }

    delete adra_api_;
    //delete timer_;
    //delete joy_state_reader_;
    init_teleop_ = false;

    rclcpp::shutdown();
    RCLCPP_INFO(this->get_logger(), "%sNode killed", log_header_.c_str());
}

bool OctoArmTeleop::auto_buffing_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    // get the speed of umbratek
    // check pid
    // tune
    // update

    auto_arm_control = !auto_arm_control;

    return true;
}

bool OctoArmTeleop::calculate_target_joint_state_()
{
    /**
     * @brief Inverse kinematics calculation logic for the target joint state
     * @param None
     * @return True if the target joint state is calculated successfully, false otherwise
     */

    double th1;
    double th2;
    double th3;
    double phi;

    double cos_th2 =
        (endeffector_x_ * endeffector_x_ + endeffector_z_ * endeffector_z_ - link1_length_ * link1_length_ -
         link2_length_ * link2_length_) /
        (2 * link1_length_ * link2_length_);

    if (abs(cos_th2) > 1.0)
    {
        if (debug_)
        {
            RCLCPP_WARN(this->get_logger(), "%sArm cannot reach the target", log_header_.c_str());
        }
        return false;
    }
    else
    {
        if (inverse_kinematics_mode_ == POSITIVE_X)
        {
            th2 = acos(cos_th2);
        }
        else if (inverse_kinematics_mode_ == NEGATIVE_X)
        {
            th2 = -1.0 * acos(cos_th2);
        }
        else
        {
            if (debug_)
            {
                RCLCPP_ERROR(this->get_logger(), "%sInvalid IK mode", log_header_.c_str());
            }
        }
    }

    th1 = atan2(endeffector_z_, endeffector_x_) -
          atan2(link2_length_ * sin(th2), link1_length_ + link2_length_ * cos(th2));
    target_joint_state_.position[1] = th1 * motor2_orientation_;
    target_joint_state_.position[2] = th2 * motor3_orientation_;
    target_joint_state_.position[3] = endeffector_head_th_ - th1 - th2;

    if (debug_)
    {
        RCLCPP_INFO(this->get_logger(), "%sEndeffector_x: %f, Endeffector_z: %f,1:%f,2:%f,3:%f,4:%f", log_header_.c_str(), endeffector_x_, endeffector_z_, target_joint_state_.position[0], target_joint_state_.position[1], target_joint_state_.position[2], target_joint_state_.position[3]);
    }
    return true;
}

void OctoArmTeleop::toggle_next_tool_()
{
    /**
     * @brief Switch to the next tool
     * @param None
     * @return None
     */
    if (init_teleop_)
    {
        if (cng_tool_cmd_)
        {
            endeffector_head_th_ += motor4_orientation_ * 1.57079632679;
            target_joint_state_.position[3] += motor4_orientation_ * 1.57079632679;
            cng_tool_cmd_ = false;
            ee_tool_next_ = false;
        }
    }
}

void OctoArmTeleop::toggle_previous_tool_()
{
    /**
     * @brief Switch to the previous tool
     * @param None
     * @return None
     **/


    if (init_teleop_)
    {
        if (cng_tool_cmd_)
        {
            endeffector_head_th_ -= motor4_orientation_ * 1.57079632679;
            target_joint_state_.position[3] -= motor4_orientation_ * 1.57079632679;
            cng_tool_cmd_ = false;
            ee_tool_prev_ = false;
        }
    }
}

void OctoArmTeleop::rotate_base_cw_()
{
    /**
     * @brief Rotate the base clockwise
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        target_joint_state_.position[0] += motor1_orientation_ * 3 / 250;
        if (target_joint_state_.position[0] > motor1_max_angle_)
        {
            target_joint_state_.position[0] = motor1_max_angle_;
        }
        else if (target_joint_state_.position[0] < motor1_min_angle_)
        {
            target_joint_state_.position[0] = motor1_min_angle_;
        }
    }
}

void OctoArmTeleop::rotate_base_ccw_()
{
    /**
     * @brief Rotate the base counter-clockwise
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        target_joint_state_.position[0] -= motor1_orientation_ * 3 / 250;
        if (target_joint_state_.position[0] > motor1_max_angle_)
        {
            target_joint_state_.position[0] = motor1_max_angle_;
        }
        else if (target_joint_state_.position[0] < motor1_min_angle_)
        {
            target_joint_state_.position[0] = motor1_min_angle_;
        }
    }
}

void OctoArmTeleop::rotate_endeffector_tool_cw_()
{
    /**
     * @brief Rotate the endeffector tool clockwise
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        endeffector_head_th_ += motor4_orientation_ * 10.0 / calc_rate_;
        target_joint_state_.position[3] += motor4_orientation_ * 10.0 / calc_rate_;
        if (target_joint_state_.position[3] > motor4_max_angle_)
        {
            endeffector_head_th_ = motor4_max_angle_;
            target_joint_state_.position[3] = motor4_max_angle_;
        }
        else if (target_joint_state_.position[3] < motor4_min_angle_)
        {
            endeffector_head_th_ = motor4_min_angle_;
            target_joint_state_.position[3] = motor4_min_angle_;
        }
    }
}

void OctoArmTeleop::rotate_endeffector_tool_ccw_()
{
    /**
     * @brief Rotate the endeffector tool counter-clockwise
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        endeffector_head_th_ -= motor4_orientation_ * 10.0 / calc_rate_;
        target_joint_state_.position[3] -= motor4_orientation_ * 10.0 / calc_rate_;
        if (target_joint_state_.position[3] > motor4_max_angle_)
        {
            endeffector_head_th_ = motor4_max_angle_;
            target_joint_state_.position[3] = motor4_max_angle_;
        }
        else if (target_joint_state_.position[3] < motor4_min_angle_)
        {
            endeffector_head_th_ = motor4_min_angle_;
            target_joint_state_.position[3] = motor4_min_angle_;
        }
    }
}

void OctoArmTeleop::increase_endeffector_height_()
{
    /**
     * @brief Increase the endeffector height
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        endeffector_x_ -= 1.0 / calc_rate_ * sin(relative_inclination_);
        endeffector_z_ += 1.0 / calc_rate_ * cos(relative_inclination_);

        if (!calculate_target_joint_state_())
        {
            endeffector_x_ += 1.0 / calc_rate_ * sin(relative_inclination_);
            endeffector_z_ -= 1.0 / calc_rate_ * cos(relative_inclination_);
        }
    }
}


void OctoArmTeleop::decrease_endeffector_height_()
{
    /**
     * @brief Decrease the endeffector height
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        endeffector_x_ += 1.0 / calc_rate_ * sin(relative_inclination_);
        endeffector_z_ -= 1.0 / calc_rate_ * cos(relative_inclination_);
        if (!calculate_target_joint_state_())
        {
            endeffector_x_ -= 1.0 / calc_rate_ * sin(relative_inclination_);
            endeffector_z_ += 1.0 / calc_rate_ * cos(relative_inclination_);
        }
    }
}

void OctoArmTeleop::increase_endeffector_depth_()
{
    /**
     * @brief Increase the endeffector depth
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        if (inverse_kinematics_mode_ == (int)POSITIVE_X)
        {
            endeffector_x_ += 1.0 / calc_rate_ * cos(relative_inclination_);
            endeffector_z_ += 1.0 / calc_rate_ * sin(relative_inclination_);
            if (!calculate_target_joint_state_())
            {
                endeffector_x_ -= 1.0 / calc_rate_ * cos(relative_inclination_);
                endeffector_z_ -= 1.0 / calc_rate_ * sin(relative_inclination_);
            }
        }
        else if (inverse_kinematics_mode_ == (int)NEGATIVE_X)
        {
            endeffector_x_ -= 1.0 / calc_rate_ * cos(relative_inclination_);
            endeffector_z_ -= 1.0 / calc_rate_ * sin(relative_inclination_);
            if (!calculate_target_joint_state_())
            {
                endeffector_x_ += 1.0 / calc_rate_ * cos(relative_inclination_);
                endeffector_z_ += 1.0 / calc_rate_ * sin(relative_inclination_);
            }
        }
        else
        {
            if (debug_)
                RCLCPP_INFO(this->get_logger(), "%sInvalid inverse kinematics mode", log_header_.c_str());
        }
    }
}

void OctoArmTeleop::decrease_endeffector_depth_()
{
    /**
     * @brief Decrease the endeffector depth
     * @param None
     * @return None
     */

    if (init_teleop_)
    {
        if (inverse_kinematics_mode_ == (int)POSITIVE_X)
        {
            endeffector_x_ -= 1.0 / calc_rate_ * cos(relative_inclination_);
            endeffector_z_ -= 1.0 / calc_rate_ * sin(relative_inclination_);
            if (!calculate_target_joint_state_())
            {
                endeffector_x_ += 1.0 / calc_rate_ * cos(relative_inclination_);
                endeffector_z_ += 1.0 / calc_rate_ * sin(relative_inclination_);
            }
        }
        else if (inverse_kinematics_mode_ == (int)NEGATIVE_X)
        {
            endeffector_x_ += 1.0 / calc_rate_ * cos(relative_inclination_);
            endeffector_z_ += 1.0 / calc_rate_ * sin(relative_inclination_);
            if (!calculate_target_joint_state_())
            {
                endeffector_x_ -= 1.0 / calc_rate_ * cos(relative_inclination_);
                endeffector_z_ -= 1.0 / calc_rate_ * sin(relative_inclination_);
            }
        }
        else
        {
             if (debug_)
                RCLCPP_INFO(this->get_logger(), "%sInvalid inverse kinematics mode", log_header_.c_str());
        }
    }
}

void OctoArmTeleop::timer_callback_()
{
    /**
     * @brief Joint state publisher timer callback
     * @param event - timer event
     * @type ros::TimerEvent
     * @return None
     * @details This function publishes the joint states and restricts the arm to the workspace.
     */

    if (init_teleop_)
    {
        if (target_joint_state_.position.size() != 4)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state size is not 5", log_header_.c_str());
            return;
        }
        if (target_joint_state_.position[0] < motor1_min_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor1 angle is out of range", log_header_.c_str());
            target_joint_state_.position[0] = motor1_min_angle_;
        }
        if (target_joint_state_.position[0] > motor1_max_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor1 angle is out of range", log_header_.c_str());
            target_joint_state_.position[0] = motor1_max_angle_;
        }
        if (target_joint_state_.position[1] < motor2_min_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor2 angle is out of range", log_header_.c_str());
            target_joint_state_.position[1] = motor2_min_angle_;
        }
        if (target_joint_state_.position[1] > motor2_max_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor2 angle is out of range", log_header_.c_str());
            target_joint_state_.position[1] = motor2_max_angle_;
        }
        if (target_joint_state_.position[2] < motor3_min_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor3 angle is out of range", log_header_.c_str());
            target_joint_state_.position[2] = motor3_min_angle_;
        }
        if (target_joint_state_.position[2] > motor3_max_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor3 angle is out of range", log_header_.c_str());
            target_joint_state_.position[2] = motor3_max_angle_;
        }
        if (target_joint_state_.position[3] < motor4_min_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor4 angle is out of range", log_header_.c_str());
            target_joint_state_.position[3] = motor4_min_angle_;
        }
        if (target_joint_state_.position[3] > motor4_max_angle_)
        {
            if (debug_)
                RCLCPP_ERROR(this->get_logger(), "%sTarget joint state motor4 angle is out of range", log_header_.c_str());
            target_joint_state_.position[3] = motor4_max_angle_;
        }

        target_joint_state_.header.stamp = this->get_clock()->now();
        joint_state_pub_->publish(target_joint_state_);

        if (sim_ == false)
        {
            int ret = adra_api_->set_pos_target(1, (float)target_joint_state_.position[0] * 100.0);
            if (ret != 0)
            {
                if (debug_)
                {
                    RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set target failed", log_header_.c_str(), 1);
                }
            }
            ret = adra_api_->set_pos_target(2, (float)target_joint_state_.position[1] * 100.0);
            if (ret != 0)
            {
                if (debug_)
                {
                    RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set target failed", log_header_.c_str(), 2);
                }
            }
            ret = adra_api_->set_pos_target(3, (float)target_joint_state_.position[2] * 100.0);
            if (ret != 0)
            {
                if (debug_)
                {
                    RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set target failed : %f", log_header_.c_str(), 3, target_joint_state_.position[2] * 100);
                }
            }
            ret = adra_api_->set_pos_target(4, (float)target_joint_state_.position[3] * 100.0);
            if (ret != 0)
            {
                if (debug_)
                {
                   RCLCPP_ERROR(this->get_logger(), "%s Actuator %d set target failed :%f", log_header_.c_str(), 4, target_joint_state_.position[3] * 100);
                }
            }
        }
    }
}

void OctoArmTeleop::teleop_timer_callback_()
{
    if (error_flag_)
    {
        diagnose_motors();
        error_flag_ = false;
    }
}

void OctoArmTeleop::joy_state_reader_callback_()
{
    /**
     * @brief Callback function for implementing the command flags
     * @param event The timer event
     * @type ros::TimerEvent
     * @return None
     */

    if (rot_base_cw_)
    {
        rotate_base_cw_();
    }
    else if (rot_base_ccw_)
    {
        rotate_base_ccw_();
    }

    if (ee_height_up_)
    {
        increase_endeffector_height_();
    }
    else if (ee_height_down_)
    {
        decrease_endeffector_height_();
    }

    if ((ee_depth_fw_) || (auto_buff_up_))
    {
        increase_endeffector_depth_();
    }
    else if ((ee_depth_bw_) || (auto_buff_down_))
    {
        decrease_endeffector_depth_();
    }

    if (ee_tool_rot_cw_)
    {
        rotate_endeffector_tool_cw_();
    }
    else if (ee_tool_rot_ccw_)
    {
        rotate_endeffector_tool_ccw_();
    }

    if (ee_tool_next_)
    {
        next_tool_();
    }
    else if (ee_tool_prev_)
    {
        previous_tool_();
    }

    // home
    if (joy_home_flag_)
    {
        // home
        home_config();
        joy_home_flag_ = false;
    }

    // work
    if (joy_work_flag_)
    {
        // work
        work_config();
        joy_work_flag_ = false;
    }
    if (joy_horizontal_flag_)
    {
        // horizontal
        horizontal_config();
        joy_horizontal_flag_ = false;
    }

    if (joy_convex90_flag_)
    {
        // convex90 pose
        convex90_config();
        joy_convex90_flag_ = false;
    }
}

void OctoArmTeleop::pid_callback_()
{
    /**
     * @brief Callback function for implementing the pid control for tool force control
     * @param event The timer event
     * @type ros::TimerEvent
     * @return None
     */

    // takes forcee value -> adds/subs x wrt to force value ->
    // need  - rough force values ie setpoints, min max limits of force, arm x-axis

    // if flag trigs from service or something
    // get setpoint and current value
    // calc pid and increase or decrease z value
    // auto_arm_control=false;
    // if (auto_arm_control)
    {

        double force_ = calculatePID(max_, min_, z_kp_, z_ki_, z_kd_, force_req_, curr_force_);
        // ROS_INFO("force: %f", force_);
    }
}

void OctoArmTeleop::auto_buff_callback_()
{
    if (auto_arm_control)
    {

        if (curr_force_ < 0.3)
        {
            auto_buff_down_ = true;
            auto_buff_up_ = false;
        }

        else if ((curr_force_ > 0.5))
        {
            auto_buff_down_ = false;
            auto_buff_up_ = true;
        }
        else
        {
            auto_buff_down_ = false;
            auto_buff_up_ = false;
        }
    }
}


double OctoArmTeleop::calculatePID(double _max, double _min, double _Kp, double _Ki, double _Kd, double _setpoint, double _pv)
{

    double curr_time_ = this->get_clock()->now().seconds();
    ;
    // Calculate error
    double _dt = (curr_time_ - prev_time_);

    double error = _setpoint - _pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - pre_error_) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max)
        output = _max;
    else if (output < _min)
        output = _min;

    // Save error to previous error
    pre_error_ = error;
    prev_time_ = curr_time_;

    //ROS_INFO("%sPIDop: %lf", log_header_.c_str(), output);

    return output;
}

// void OctoArmTeleop::pid_tune_callback(octo_arm_teleop::pidConfig &config, uint32_t level)
// {
//     z_kp_ = config.kp;
//     z_ki_ = config.ki;
//     z_kd_ = config.kd;
//     min_ = config.min;
//     max_ = config.max;
//     force_req_ = config.req_force;
// }



int main(int argc, char **argv)
{
  // std::cout<<"hi";

  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctoArmTeleop>(rclcpp::NodeOptions());
  try
  {
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "Exception thrown: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}