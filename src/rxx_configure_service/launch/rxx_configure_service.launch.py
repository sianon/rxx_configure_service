import launch
import launch_ros

def generate_launch_description():
    action_declare_calibration_path = launch.actions.DeclareLaunchArgument('calibration_path', default_value="aaaa")
    action_declare_3d_model_path = launch.actions.DeclareLaunchArgument('3d_models_path', default_value="/home/rxx/jk/amodels/")
    action_declare_couchdb_addr = launch.actions.DeclareLaunchArgument('couchdb_addr', default_value="/home/rxx/jk/amodels/")
    action_declare_couchdb_db_name = launch.actions.DeclareLaunchArgument('couchdb_db_name', default_value="/home/rxx/jk/amodels/")

    rxx_configure_service_node = launch_ros.actions.Node(
        package='rxx_configure_service',
        executable="rxx_configure_service",
        output='screen',
        parameters=[{'calibration_path': launch.substitutions.LaunchConfiguration('calibration_path', default="/home/rxx/rxx_ws/src/rxx_calibration/hand_eye_cali/cali_source_data")},
                    {'3d_models_path': launch.substitutions.LaunchConfiguration('3d_models_path', default="/home/rxx/jk/amodels/")},
                    {'couchdb_addr': launch.substitutions.LaunchConfiguration('couchdb_addr', default="http://10.11.2.46:5984")},
                    {'couchdb_db_name': launch.substitutions.LaunchConfiguration('couchdb_db_name', default="/rxx/")},
                    {'http_port': 10001}
                    ]
    )

    launch_description = launch.LaunchDescription([
        rxx_configure_service_node,
        action_declare_calibration_path,
        action_declare_3d_model_path,
        action_declare_couchdb_addr,
        action_declare_couchdb_db_name
    ])
    return launch_description
