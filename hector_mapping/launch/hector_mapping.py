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

    UAV_TYPE=os.getenv('UAV_TYPE')
    UAV_NAME=os.getenv('UAV_NAME')

    namespace=UAV_NAME
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_hector',
        package='rclcpp_components',
        executable='component_container_mt',
        # prefix=['xterm -e gdb -ex run --args'],
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='hector_mapping::HectorMappingRos',
                namespace=namespace,
                name='hector',

                parameters=[
                    pkg_share_path + '/config/params.yaml',
                    {"tf_map_scanmatch_transform_f": UAV_NAME+"/scanmatcher_frame"},
                    {"map_frame": UAV_NAME+"/hector_map"},
                    {"base_frame": UAV_NAME+"/fcu_untilted"},
                    {"odom_frame": UAV_NAME+"/fcu"},
                    {"scan_topic": UAV_NAME+"/rplidar/scan"},
                ],
                # remappings=[
                #     # topics
                #     ("~/topic_out", "~/topic"),
                # ],
            ),
        ],
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')]
    ))

    return ld
