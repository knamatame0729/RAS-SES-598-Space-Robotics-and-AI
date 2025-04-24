from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Static TF publisher for camera to base link transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),  

        # RTAB-Map node
        Node(
            package='rtabmap_slam',     #rtabmap_slam
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            #arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                
                # RTAB-Map parameters
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'sync_queue_size': 10,
                
                # Odometry parameters
                'odom_frame_id': 'odom',
                'subscribe_odom_info': False,
                'odom_tf_angular_variance': 0.01,
                'odom_tf_linear_variance': 0.001,
                
                # Visual odometry parameters
                'visual_odometry': False,  # Using PX4 odometry instead
                
                # Mapping parameters
                'grid_cell_size': 0.05,
                'grid_size': 20.0,
                'optimize_from_graph_end': True,
                'optimizer_iterations': 100,
                
                # Loop closure parameters
                'loop_closure_activated': True,
                'loop_closure_restriction_type': 0,
                'loop_closure_min_inliers': 20,
                
                # Memory management
                'memory_management': True,
                'max_cloud_size': 50000,
                'min_cluster_size': 100
            }],
            remappings=[
                # Camera topics
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('rgb/camera_info', '/drone/front_rgb/camera_info'),
                
                # Odometry from PX4
                ('odom', '/fmu/out/vehicle_odometry'),
                
                # Output topics
                ('grid_map', 'map'),
                ('mapData', 'mapData'),
                ('mapPath', 'mapPath'),
                ('cloud_map', 'cloud_map')
            ]
        ),

        # RTAB-Map point cloud generation
        Node(
            package='rtabmap_util',            # rtabmap_util
            executable='point_cloud_xyz',
            name='point_cloud_xyz',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'decimation': 4,      # 2 -> 4,
                'voxel_size': 0.02,  # 0.03 -> 0.02
                'max_depth': 4.0,
                'min_depth': 0.4,
            }],
            remappings=[
                ('depth/image', '/drone/front_depth'),
                ('depth/camera_info', '/drone/front_depth/camera_info'),
                ('cloud', 'cloud_xyz')
            ]
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'approx_sync': True,
                'Gui': True,
                'subscribe_rgb': True,
                'sbuscribe_depth': True,
            }],
            remappings=[
                ('rgb/image', '/drone/front_rgb'),
                ('depth/image', '/drone/front_depth'),
                ('rgb/camera_info', '/drone/front_rgb/camera_info'),
                ('odom', '/fmu/out/vehicle_odometry'),
                ('grid_map', 'map'),
                ('mapData', 'mapData'),
                ('cloud_map', 'cloud_map')
            ]
        ),

        # Log info
        LogInfo(
            msg="RTAB-Map launched with drone configuration"
        )
    ]) 