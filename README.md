# OctoArm Teleoperation — ROS 2 Package

**OctoArm Teleop** is a modular and extensible ROS 2 package designed for real-time teleoperation and control of the OctoArm robotic manipulator. It integrates joystick input, PID-based motion control, and configurable kinematic behaviors to enable seamless interaction with both simulated and physical robotic platforms.

---

## 📦 Overview

This package implements a ROS 2 node that provides:

- Joystick-based teleoperation using `sensor_msgs/msg/Joy`
- PID control for precise end-effector height manipulation
- Support for real hardware (via serial communication) and simulation
- Configurable joint limits, home/workspace presets, and teleop modes
- Timer-driven architecture for deterministic control execution

---

## 🔧 Dependencies

- ROS 2 (Humble or newer)
- utapi for low-level hardware communication
- `joy` package for joystick input (e.g., Xbox or Logitech controllers)

---

## 📁 Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/octobotics/octo_arm_teleop_ros2.git
cd ..
colcon build --packages-select octo_arm_teleop_ros2
source install/setup.bash
```

---

## 🚀 Launching the Node

```bash
ros2 launch octo_arm_teleop_ros2 octo_arm_teleop.launch.py
```

---

## 📡 Topics

### 🔽 Subscribed Topics

| Topic Name                         | Message Type              | Description                                 |
|-----------------------------------|---------------------------|---------------------------------------------|
| `/joy`                            | `sensor_msgs/msg/Joy`     | Joystick input                              |
| `/octo_arm_teleop/relative_inclination` | `std_msgs/msg/Float64` | Relative inclination from external node     |
| `/toggle_joy`                     | `std_msgs/msg/Bool`       | Toggle joystick control                     |
| `/send_force_feedback`            | `std_msgs/msg/Float32`    | Force feedback input                        |

### 🔼 Published Topics

| Topic Name             | Message Type                              | Description                                 |
|------------------------|-------------------------------------------|---------------------------------------------|
| `/joint_states`        | `sensor_msgs/msg/JointState`              | Current joint positions (for monitoring)    |
| `/send_arm_status`     | `stm_client/msg/Sendadrastatus`           | Custom status feedback from the robot       |

---

## 🛠️ Services

### 🧭 Service Servers (Provided by Node)

| Service Name                     | Service Type                        | Description                                 |
|----------------------------------|-------------------------------------|---------------------------------------------|
| `/octo_arm_teleop/init_teleop`  | `std_srvs/srv/Trigger`              | Initialize teleoperation                    |
| `/octo_arm_teleop/stop_teleop`  | `std_srvs/srv/Trigger`              | Stop teleoperation                          |
| `/octo_arm_teleop/set_ik_mode`  | `stm_client/srv/SetIkMode`          | Set inverse kinematics mode                 |
| `/gui_adra_status_srv`          | `stm_client/srv/GUIadrastat`        | Provide GUI feedback on Adra status         |
| `/octo_arm_teleop/reset_arm`    | `std_srvs/srv/Trigger`              | Reset robot arm to initial state            |
| `/octo_arm_teleop/auto_buff_pid`| `std_srvs/srv/Trigger`              | Automatically tune or reset PID buffers     |

### 🤖 Service Clients (Used by Node)

| Service Name                | Service Type                    | Description                                   |
|-----------------------------|----------------------------------|-----------------------------------------------|
| `/relay_toggle_channel`    | `stm_client/srv/Relaycontrol`   | Toggle relay power/control line               |
| `/reset_force_feedback_`   | `std_srvs/srv/Trigger`          | Reset force feedback mechanism                |
| `/toggle_force_feedback_`  | `std_srvs/srv/SetBool`          | Enable or disable force feedback system       |
| `/octo_arm_teleop/init_teleop` (client) | `std_srvs/srv/Trigger` | Also called internally for re-init           |


---

## 🔌 Parameters

All parameters can be declared via CLI or configuration files. Below are key configurable parameters:

| Parameter                                | Type     | Default       | Description                                               |
|------------------------------------------|----------|---------------|-----------------------------------------------------------|
| `port_name`                              | string   | `/dev/ttyUSB0`| Serial port for hardware communication                    |
| `baud_rate`                              | int      | `921600`      | Baud rate for Adra motor serial communication             |
| `sim`                                    | bool     | `false`       | Enables simulation mode (disables hardware communication) |
| `debug`                                  | bool     | `true`        | Enables verbose logging                                   |
| `publish_rate`                           | double   | `300.0`       | Rate (Hz) for publishing joint commands                   |
| `calc_rate`                              | double   | `400.0`       | Rate (Hz) for PID and teleop update loops                 |
| `ik_mode`                                | int      | `0`           | Inverse kinematics mode (0: manual, 1: auto)              |
| `joy_topic`                              | string   | `/joy`        | Joystick input topic                                      |
| `joint_state_topic`                      | string   | `/octo_adra_ros/cmd_joint_states` | Target joint state publisher               |
| `base_orientation` to `tool_orientation` | double   | configurable  | Initial joint positions for home and work states          |

---

## ⏱ Timers and Execution Loops

The node uses ROS 2 timers to ensure non-blocking control logic:

- `timer_callback_`: General loop for high-level behaviors
- `teleop_timer_callback_`: Joystick input interpretation
- `joy_state_reader_callback_`: State buffering of joystick input
- `pid_callback_`: PID-based height control
- `auto_buff_callback_`: Automated recovery/buffer behaviors

---

## 🧼 Resource Management

- `adra_api_` (hardware interface) is allocated only in non-simulated modes and safely released in the destructor.
- Timers are managed via `rclcpp::TimerBase::SharedPtr`, ensuring automatic cleanup.

---

## 👤 Contributors

**Nilanjan Chowdhury**
**Priyanshu Srivastava**

---
