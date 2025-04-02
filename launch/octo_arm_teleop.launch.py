import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        DeclareLaunchArgument('port_name', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('publish_rate', default_value='300.0'),
        DeclareLaunchArgument('calc_rate', default_value='600.0'),
        DeclareLaunchArgument('motor1_home', default_value='0.0'),
        DeclareLaunchArgument('motor2_home', default_value='-0.0'),
        DeclareLaunchArgument('motor3_home', default_value='0.8'),
        DeclareLaunchArgument('motor4_home', default_value='0.0'),
        DeclareLaunchArgument('motor5_home', default_value='0.3'),
        DeclareLaunchArgument('joint_state_topic', default_value='/octo_adra_ros/cmd_joint_states'),
        DeclareLaunchArgument('joy_topic', default_value='joy'),
        DeclareLaunchArgument('link1_length', default_value='0.28'),
        DeclareLaunchArgument('link2_length', default_value='0.275'),
        DeclareLaunchArgument('motor1_orientation', default_value='1.0'),
        DeclareLaunchArgument('motor2_orientation', default_value='1.0'),
        DeclareLaunchArgument('motor3_orientation', default_value='1.0'),
        DeclareLaunchArgument('motor4_orientation', default_value='1.0'),
        DeclareLaunchArgument('motor1_min_angle', default_value='-1.75'),
        DeclareLaunchArgument('motor2_min_angle', default_value='-3.14'),
        DeclareLaunchArgument('motor3_min_angle', default_value='-3.14'),
        DeclareLaunchArgument('motor4_min_angle', default_value='-6.28'),
        DeclareLaunchArgument('motor1_max_angle', default_value='1.57'),
        DeclareLaunchArgument('motor2_max_angle', default_value='3.14'),
        DeclareLaunchArgument('motor3_max_angle', default_value='3.14'),
        DeclareLaunchArgument('motor4_max_angle', default_value='6.28'),
        DeclareLaunchArgument('relative_inclination', default_value='0.0'),
        DeclareLaunchArgument('ik_mode', default_value='0'),

       
        Node(
            package='octo_arm_teleop_ros2',
            executable='octo_arm_teleop_node',
            name='octo_arm_teleop_node',
            output='screen',
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'sim': LaunchConfiguration('sim'),
                'debug': LaunchConfiguration('debug'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'calc_rate': LaunchConfiguration('calc_rate'),
                'motor1/home_angle': LaunchConfiguration('motor1_home'),
                'motor2/home_angle': LaunchConfiguration('motor2_home'),
                'motor3/home_angle': LaunchConfiguration('motor3_home'),
                'motor4/home_angle': LaunchConfiguration('motor4_home'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'joy_topic': LaunchConfiguration('joy_topic'),
                'link1_length': LaunchConfiguration('link1_length'),
                'link2_length': LaunchConfiguration('link2_length'),
                'motor1/orientation': LaunchConfiguration('motor1_orientation'),
                'motor2/orientation': LaunchConfiguration('motor2_orientation'),
                'motor3/orientation': LaunchConfiguration('motor3_orientation'),
                'motor4/orientation': LaunchConfiguration('motor4_orientation'),
                'motor1/min_angle': LaunchConfiguration('motor1_min_angle'),
                'motor2/min_angle': LaunchConfiguration('motor2_min_angle'),
                'motor3/min_angle': LaunchConfiguration('motor3_min_angle'),
                'motor4/min_angle': LaunchConfiguration('motor4_min_angle'),
                'motor1/max_angle': LaunchConfiguration('motor1_max_angle'),
                'motor2/max_angle': LaunchConfiguration('motor2_max_angle'),
                'motor3/max_angle': LaunchConfiguration('motor3_max_angle'),
                'motor4/max_angle': LaunchConfiguration('motor4_max_angle'),
                'relative_inclination': LaunchConfiguration('relative_inclination'),
                'ik_mode': LaunchConfiguration('ik_mode'),
            }],
            respawn=True,
            respawn_delay=5
        )
    ])
