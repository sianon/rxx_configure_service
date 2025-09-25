import launch
import launch_ros

def generate_launch_description():
    # action_declare_arg_max_spped = launch.actions.DeclareLaunchArgument('launch_max_speed', default_value='2.0')
    rxx_configure_service_node = launch_ros.actions.Node(
        package='rxx_configure_service',
        executable="rxx_configure_service",
        output='screen'
    )

    launch_description = launch.LaunchDescription([rxx_configure_service_node])
    return launch_description
