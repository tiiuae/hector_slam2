import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "hector_mapping"
    pkg_share_path = get_package_share_directory(pkg_name)

    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

    UAV_TYPE=os.getenv('UAV_TYPE')
    UAV_NAME=os.getenv('UAV_NAME')

    namespace=UAV_NAME
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_hector_mapping',
        package='rclcpp_components',
        executable='component_container_mt',
        # prefix=['xterm -e gdb -ex run --args'],
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='hector_mapping::HectorMappingRos',
                namespace=namespace,
                name='hector_mapping',

                parameters=[
                    pkg_share_path + '/config/params.yaml',
                    {"tf_map_scanmatch_transform_f": UAV_NAME+"/scanmatcher_frame"},
                    {"map_frame": "hector_origin"},
                    {"base_frame": "fcu"},
                    {"odom_frame": "local_odom"},
                    {"scan_topic": "/uav1/rplidar/scan"},
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                # remappings=[
                #     # topics
                #     ("~/topic_out", "~/topic"),
                # ],
            ),
        ],
        output='screen',
        # emulate_tty=True,
        # arguments=[('__log_level:=debug')],
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
