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
                    package="rtabmap_sync",
                    plugin="rtabmap_sync::RGBDSync",
                    name="rgbd_sync",
                    remappings=[('rgb/image', 'oakd/rgb/image_raw'), ('depth/image', 'oakd/stereo/image_raw'), ('rgb/camera_info', 'oakd/rgb/camera_info'), ('rgbd_image', 'rgbd_image')],
                    parameters=[{"approx_sync": True, "approx_sync_max_interval": 0.0, "queue_size": 10, "depth_scale": 1.0, "decimation": 1}]   
                )
            ])
    

    node4 = LoadComposableNodes(
            target_container="oakd_container",
            composable_node_descriptions=[
                ComposableNode(
                    package='rtabmap_slam',
                    plugin='rtabmap_slam::CoreWrapper',
                    name='rtabmap',
                    parameters=[{
                                "subscribe_rgbd": False,
                                "subscribe_depth": True,
                                "subscribe_rgb": True,
                                "subscribe_stereo": False,
                                "subscribe_scan": False,
                                "subscribe_scan_cloud": False,
                                "subscribe_scan_descriptor": False,
                                "subscribe_user_data": False,
                                "frame_id": 'oakd_link',
                                "odom_frame_id": "odom",
                                "map_frame_id": "map",
                                "publish_tf": False, 
                                "odom_sensor_sync": False, #ska kanske vara true
                                "wait_for_transform_duration": 0.5,
                                "subscribe_odom_info": False,
                                "approx_sync": True, 
                                "Rtabmap/DetectionRate": "3.5", 
                                "queue_size": 10,
                                "gen_scan": False,
                                "gen_depth": False,
                                #"args": "--delete_db_on_start", #kanske fungerar
                            }],
                    remappings=[('depth/image', 'oakd/stereo/image_raw'),('rgb/image', 'oakd/rgb/image_rect'), ('rgb/camera_info', 'oakd/rgb/camera_info'), ('imu', 'dontUseIt')],
                ),
            ],
        )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node)
    ld.add_action(node2)
    #ld.add_action(node3)
    ld.add_action(node4)

    return ld
