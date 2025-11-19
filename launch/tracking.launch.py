import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the octomap server for change tracking."""
    
    # Declare the launch argument for the input cloud topic
    cloud_in_topic_arg = DeclareLaunchArgument(
        'cloud_in_topic',
        default_value='camera/depth/points',
        description='Point cloud topic to subscribe to for OctoMap.'
    )

    # Declare launch arguments for output topics
    octomap_full_topic_arg = DeclareLaunchArgument(
        'octomap_full_topic',
        default_value='mapskit/voxelmap_full',
        description='Topic for the full OctoMap.'
    )
    octomap_binary_topic_arg = DeclareLaunchArgument(
        'octomap_binary_topic',
        default_value='mapskit/voxelmap_binary',
        description='Topic for the binary OctoMap.'
    )
    octomap_centers_topic_arg = DeclareLaunchArgument(
        'octomap_centers_topic',
        default_value='mapskit/voxelmap_centers',
        description='Topic for the OctoMap point cloud centers.'
    )

    # Get the launch configuration
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    octomap_full_topic = LaunchConfiguration('octomap_full_topic')
    octomap_binary_topic = LaunchConfiguration('octomap_binary_topic')
    octomap_centers_topic = LaunchConfiguration('octomap_centers_topic')

    # Default parameters for the octomap_server node
    default_params = {
        # Map Resolution and Frames
        'resolution': 0.05,
        'frame_id': 'odom',
        'base_frame_id': 'base_link',
        'use_sim_time': False,
        
        # TF Buffer for handling timing issues
        'tf_buffer_duration': 2.0,
        
        # Sensor Configuration
        'sensor_model.max_range': 5.0,
        
        # Disable ground filtering for better change detection
        'ground_filter.enable': False,
        'point_cloud_min_z': -100.0,
        'point_cloud_max_z': 100.0,
        
        # TRACKING CONFIGURATION
        'track_changes': True,
        'listen_changes': False,
        'topic_changes': '/mapskit/changeset',
        'change_id_frame': 'odom',
    }

    # Node for the octomap server
    octomap_server_node = Node(
        package='octomap_server',
        executable='tracking_octomap_server_node',
        name='mapskit_node',
        # output='screen',
        parameters=[default_params],
        remappings=[
            ('cloud_in', cloud_in_topic),
            ('octomap_full', octomap_full_topic),
            ('octomap_binary', octomap_binary_topic),
            ('octomap_point_cloud_centers', octomap_centers_topic)
        ]
    )

    return LaunchDescription([
        cloud_in_topic_arg,
        octomap_full_topic_arg,
        octomap_binary_topic_arg,
        octomap_centers_topic_arg,
        octomap_server_node
    ])
