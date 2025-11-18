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
        default_value='/depth/points',
        description='Point cloud topic to subscribe to for OctoMap.'
    )

    # Get the launch configuration
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')

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
        'save_directory': os.getenv('OCTOMAP_SAVE_DIR', './'),
        'track_changes': True,
        'listen_changes': False,
        'topic_changes': '/mapskit/changeset',
        'change_id_frame': 'odom',
    }

    # Node for the octomap server
    octomap_server_node = Node(
        package='octomap_server',
        executable='tracking_octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[default_params],
        remappings=[
            ('cloud_in', cloud_in_topic)
        ]
    )

    return LaunchDescription([
        cloud_in_topic_arg,
        octomap_server_node
    ])
