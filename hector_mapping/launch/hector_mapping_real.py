import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import sys


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "hector_mapping"
    pkg_share_path = get_package_share_directory(pkg_name)

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_hector_mapping',
        package='rclcpp_components',
        executable='component_container',
        # prefix=['xterm -e gdb -ex run --args'],
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='hector_mapping::HectorMappingRos',
                namespace=namespace,
                name='hector_mapping',

                parameters=[
                    pkg_share_path + '/config/real.yaml',
                    {"map_frame": DRONE_DEVICE_ID + "/hector_origin"},
                    {"base_frame": DRONE_DEVICE_ID + "/fcu"},
                    {"odom_frame": DRONE_DEVICE_ID + "/ned_fcu"},
                    {"scan_topic": "/"+DRONE_DEVICE_ID+"/rplidar/scan"},
                    {"sys_msg_topic": "syscommand"},
                    {"pose_update_topic": "poseupdate"},
                    {"tf_map_scanmatch_transform_frame_name": "scanmatcher_frame"},
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("~/reset_hector", "~/reset_hector"),
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        # emulate_tty=True,
        # arguments=[('__log_level:=debug')],
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
