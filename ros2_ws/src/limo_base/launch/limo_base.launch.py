import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
    #                                          description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttylimo',
                                         description='usb bus name, e.g. ttyUSB0')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')
    odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='False',
                                           description='Odometry topic name')

    # is_scout_mini_arg = DeclareLaunchArgument('is_scout_mini', default_value='false',
    #                                       description='Scout mini model')
    # is_omni_wheel_arg = DeclareLaunchArgument('is_omni_wheel', default_value='false',
    #                                       description='Scout mini omni-wheel model')

    # simulated_robot_arg = DeclareLaunchArgument('simulated_robot', default_value='false',
    #                                                description='Whether running with simulator')
    steering_scale_arg = DeclareLaunchArgument('steering_scale', default_value='1.06',
                                               description='inner wheel angle (rad) per body steering value, measured on limo2 (official 2.47)')
    max_inner_angle_arg = DeclareLaunchArgument('max_inner_angle', default_value='0.48869',
                                                 description='inner wheel limit in rad, official 28 deg')
    cmd_timeout_arg = DeclareLaunchArgument('cmd_timeout', default_value='0.5',
                                            description='seconds without /cmd_vel before stopping, 0.0 disables')
    yolo_ready_arg = DeclareLaunchArgument('require_yolo_ready', default_value='false',
                                           description='hold automatic commands at zero until /yolo/ready')

    sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
                                                 description='Simulation control loop update rate')
    
    limo_base_node = launch_ros.actions.Node(
        package='limo_base',
        executable='limo_base',  #foxy executable='limo_base',
        output='screen',
        emulate_tty=True,
        parameters=[{
                # 'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'port_name': launch.substitutions.LaunchConfiguration('port_name'),                
                'odom_frame': launch.substitutions.LaunchConfiguration('odom_frame'),
                'base_frame': launch.substitutions.LaunchConfiguration('base_frame'),
                'odom_topic_name': launch.substitutions.LaunchConfiguration('odom_topic_name'),
                'pub_odom_tf': launch.substitutions.LaunchConfiguration('pub_odom_tf'),
                'steering_scale': launch.substitutions.LaunchConfiguration('steering_scale'),
                'max_inner_angle': launch.substitutions.LaunchConfiguration('max_inner_angle'),
                'cmd_timeout': launch.substitutions.LaunchConfiguration('cmd_timeout'),
                'require_yolo_ready': launch.substitutions.LaunchConfiguration('require_yolo_ready'),
                'control_rate': launch.substitutions.LaunchConfiguration('control_rate'),
        }],
        namespace='limo'
    )

    return LaunchDescription([
        # use_sim_time_arg,
        port_name_arg,        
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        odom_tf_arg,
        steering_scale_arg,
        max_inner_angle_arg,
        cmd_timeout_arg,
        yolo_ready_arg,
        # is_scout_mini_arg,
        # is_omni_wheel_arg,
        # simulated_robot_arg,
        sim_control_rate_arg,
        limo_base_node
    ])
