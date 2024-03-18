from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
#    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    this_pkg_share = get_package_share_directory('road_navigate')

    camera = LaunchConfiguration('camera')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    ARGUMENTS = [
        DeclareLaunchArgument('camera', default_value='oakd_pro'),
        DeclareLaunchArgument('params_file', default_value=[PathJoinSubstitution([this_pkg_share, 'config', 'rtabmap']), '.yaml']),
        DeclareLaunchArgument('namespace', default_value='',description='Robot namespace')
    ]

    parameters = [
        {
            "frame_id": 'oakd_link',
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
	        "queue_size": 10,
            "approx_sync_max_interval": 0.015,
        }
    ]

    remappings = [
        ("rgb/image", "oakd/rgb/image_rect"),
        ("rgb/camera_info", "oakd/rgb/camera_info"),
        ("depth/image", "oakd/stereo/image_raw"),
        ("odom","rgb/odom"),
        ('imu', 'rgb/imu'),
        ('tf', 'rgb/tf'),
    ]
    



    namespaced_param_file = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    node = ComposableNodeContainer(
            name='oakd_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='depthai_ros_driver',
                        plugin='depthai_ros_driver::Camera',
                        name='oakd',
                        parameters=[namespaced_param_file],
                    ),
            ],
            output='screen',
        )
    

    node2 = LoadComposableNodes(
            #condition=IfCondition(LaunchConfiguration("rectify_rgb")),
            target_container="oakd_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    remappings=[('image', 'oakd/rgb/image_raw'),
                                ('camera_info', 'oakd/rgb/camera_info'),
                                ('image_rect', 'oakd/rgb/image_rect'),
                                ('image_rect/compressed', 'oakd/rgb/image_rect/compressed'),
                                ('image_rect/compressedDepth', 'oakd/rgb/image_rect/compressedDepth'),
                                ('image_rect/theora', 'oakd/rgb/image_rect/theora')]
                )
            ])
        
    node3 = LoadComposableNodes(
            target_container="oakd_container",
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_odom',
                    plugin='rtabmap_odom::RGBDOdometry',
                    name='rgbd_odometry',
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
        )

    node4 = LoadComposableNodes(
            target_container="oakd_container",
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_slam',
                    plugin='rtabmap_slam::CoreWrapper',
                    name='rtabmap',
                    parameters=parameters,
                    remappings=remappings,
                ),
            ],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node4)

    return ld