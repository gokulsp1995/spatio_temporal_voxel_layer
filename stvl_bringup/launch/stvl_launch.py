"""STVL evaluation bringup on head360.

TF chain this launch builds:
  map --(AMCL, or fixed fallback)--> odom --(KISS-ICP)--> base_link --(fixed mount)--> livox_frame

Usage:
  ros2 launch stvl_bringup stvl_launch.py                                   # AMCL + global costmap
  ros2 launch stvl_bringup stvl_launch.py params:=stvl_local.yaml           # local rolling costmap
  ros2 launch stvl_bringup stvl_launch.py params:=voxel_baseline.yaml       # Nav2 VoxelLayer baseline
  ros2 launch stvl_bringup stvl_launch.py use_amcl:=false start_x:=-3.0 start_y:=-2.0 start_yaw:=3.14
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('stvl_bringup')

    livox_launch = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'launch_ROS2', 'stvl_MID360s_launch.py')

    costmap_params = PathJoinSubstitution(
        [FindPackageShare('stvl_bringup'), 'config', LaunchConfiguration('params')])

    amcl_params = os.path.join(bringup_share, 'config', 'amcl.yaml')

    use_amcl = LaunchConfiguration('use_amcl')

    return LaunchDescription([
        # ---------------- Launch arguments ----------------
        DeclareLaunchArgument(
            'params', default_value='stvl_global.yaml',
            description='Costmap config file in stvl_bringup/config'),
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser(
                '~/stvl_ws/map_static/maps/office_static.yaml'),
            description='Static map yaml (.pgm)'),
        DeclareLaunchArgument(
            'use_amcl', default_value='true',
            description='true: AMCL publishes map->odom. '
                        'false: fixed map->odom from start_x / start_y / start_yaw'),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),

        # ---------------- 1. Lidar driver: PointCloud2 on /livox/lidar ----------------
        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch)),

        # ---------------- 2. Fixed lidar mount (from the URDF) ----------------
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_base_to_lidar',
             arguments=['--x', '0.23', '--y', '0', '--z', '0.291',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']),

        # ---------------- 3. Lidar odometry: odom -> base_link ----------------
        # odom starts where base_link is when this node starts (stance matters!).
        Node(package='kiss_icp', executable='kiss_icp_node', name='kiss_icp_node',
             output='screen',
             remappings=[('pointcloud_topic', '/livox/lidar')],
             parameters=[{
                 'base_frame': 'base_link',
                 'lidar_odom_frame': 'odom',
                 'publish_odom_tf': True,
                 # Check: `ros2 run tf2_ros tf2_echo odom base_link` must print a
                 # transform. If odom ends up as a child of base_link, set True.
                 'invert_odom_tf': False,
                 'data.min_range': 0.5,      # indoor starting guesses, tune if unstable
                 'data.max_range': 20.0,
                 'mapping.voxel_size': 0.2,
             }]),

        # ---------------- 4. 2D slice of the cloud for AMCL ----------------
        # Heights are relative to base_link (body centre), so the slice follows the body.
        Node(package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='cloud_to_scan', output='screen',
             remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')],
             parameters=[{
                 'target_frame': 'base_link',
                 'transform_tolerance': 0.2,
                 'min_height': -0.2,
                 'max_height': 0.6,
                 'angle_min': -3.14159,
                 'angle_max': 3.14159,
                 'angle_increment': 0.0087,
                 'range_min': 0.3,
                 'range_max': 10.0,
             }],
             condition=IfCondition(use_amcl)),

        # ---------------- 5. Static map ----------------
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map')}]),

        # ---------------- 6a. AMCL: map -> odom, corrected continuously ----------------
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[amcl_params],
             condition=IfCondition(use_amcl)),

        # ---------------- 6b. Fallback: fixed map -> odom (start pose on the map) ----------------
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_map_to_odom',
             arguments=['--x', LaunchConfiguration('start_x'),
                        '--y', LaunchConfiguration('start_y'), '--z', '0',
                        '--roll', '0', '--pitch', '0',
                        '--yaw', LaunchConfiguration('start_yaw'),
                        '--frame-id', 'map', '--child-frame-id', 'odom'],
             condition=UnlessCondition(use_amcl)),

        # ---------------- 7. Costmap node hosting STVL ----------------
        Node(package='nav2_costmap_2d', executable='nav2_costmap_2d',
             name='stvl_costmap', output='screen',
             parameters=[costmap_params]),

        # ---------------- 8. One lifecycle manager, started in order ----------------
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[{'autostart': True,
             'bond_timeout': 10.0,
             'node_names': ['map_server', 'amcl', 'stvl_costmap']}],
             condition=IfCondition(use_amcl)),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[{'autostart': True,
             'bond_timeout': 10.0,
                          'node_names': ['map_server', 'stvl_costmap']}],
             condition=UnlessCondition(use_amcl)),
    ])