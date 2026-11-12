"""Image hub (clothoid_image_hub::ImageHub): small camera topics for the Python nodes.

    ros2 launch clothoid_image_hub hub.launch.py                          # own container (PC, bag)
    ros2 launch clothoid_image_hub hub.launch.py container:=/camera/camera_container   # car
With `container` it is loaded into the camera driver's container and gets the raw image
intra-process; check the name on the car with `ros2 component list`. bringup includes this file
(use_image_hub), see docs/PIPELINE_LOAD_PLAN.md 0.2.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def nodes(context):
        params = [PathJoinSubstitution([FindPackageShare('clothoid_image_hub'), 'config', 'hub.yaml']),
                  PathJoinSubstitution([FindPackageShare('clothoid_lane'), 'config',
                                        ['camera_', LaunchConfiguration('camera'), '.yaml']]),
                  {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        hub = ComposableNode(package='clothoid_image_hub', plugin='clothoid_image_hub::ImageHub',
                             name='image_hub', parameters=params,
                             extra_arguments=[{'use_intra_process_comms': True}])
        container = LaunchConfiguration('container').perform(context)
        if container:
            return [LoadComposableNodes(target_container=container, composable_node_descriptions=[hub])]
        return [ComposableNodeContainer(name='image_hub_container', namespace='',
                                        package='rclcpp_components', executable='component_container',
                                        composable_node_descriptions=[hub], output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument('container', default_value='',
                              description="camera container to load into; '' = own container"),
        DeclareLaunchArgument('camera', default_value='limo2',
                              description='clothoid_lane config/camera_<camera>.yaml for the BEV warp'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
