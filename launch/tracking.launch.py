import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Get the launch configuration
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    voxelmap_full_topic = LaunchConfiguration('voxelmap_full_topic')
    voxelmap_binary_topic = LaunchConfiguration('voxelmap_binary_topic')
    voxelmap_centers_topic = LaunchConfiguration('voxelmap_centers_topic')
    
    params_file = LaunchConfiguration('params_file').perform(context)

    # Default parameters for the voxelmap_server node
    default_params = {
        # Map Resolution and Frames
        'resolution': LaunchConfiguration('resolution'),
        'frame_id': LaunchConfiguration('frame_id'),
        'base_frame_id': LaunchConfiguration('base_frame_id'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        
        # TF Buffer for handling timing issues
        'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
        
        # Sensor Configuration
        'sensor_model.max_range': LaunchConfiguration('sensor_model_max_range'),
        
        # Disable ground filtering for better change detection
        'ground_filter.enable': LaunchConfiguration('ground_filter_enable'),
        'point_cloud_min_z': LaunchConfiguration('point_cloud_min_z'),
        'point_cloud_max_z': LaunchConfiguration('point_cloud_max_z'),
        
        # TRACKING CONFIGURATION
        'track_changes': LaunchConfiguration('track_changes'),
        'listen_changes': LaunchConfiguration('listen_changes'),
        'topic_changes': LaunchConfiguration('topic_changes'),
        'change_id_frame': LaunchConfiguration('change_id_frame'),
    }

    parameters = [default_params]
    if params_file:
        parameters = [params_file]

    # Node for the voxelmap server
    voxelmap_server_node = Node(
        package='octomap_server',
        executable='tracking_octomap_server_node',
        name='mapskit_voxel_server',
        # output='screen',
        parameters=parameters,
        remappings=[
            ('cloud_in', cloud_in_topic),
            ('octomap_full', voxelmap_full_topic),
            ('octomap_binary', voxelmap_binary_topic),
            ('octomap_point_cloud_centers', voxelmap_centers_topic)
        ]
    )

    return [voxelmap_server_node]

def generate_launch_description():
    """Launch the voxelmap server for change tracking."""
    
    # Declare the launch argument for the input cloud topic
    cloud_in_topic_arg = DeclareLaunchArgument(
        'cloud_in_topic',
        default_value='camera/depth/points',
        description='Point cloud topic to subscribe to for OctoMap.'
    )

    # Declare launch arguments for output topics
    voxelmap_full_topic_arg = DeclareLaunchArgument(
        'voxelmap_full_topic',
        default_value='mapskit/voxelmap_full',
        description='Topic for the full OctoMap.'
    )
    voxelmap_binary_topic_arg = DeclareLaunchArgument(
        'voxelmap_binary_topic',
        default_value='mapskit/voxelmap_binary',
        description='Topic for the binary OctoMap.'
    )
    voxelmap_centers_topic_arg = DeclareLaunchArgument(
        'voxelmap_centers_topic',
        default_value='mapskit/voxelmap_centers',
        description='Topic for the OctoMap point cloud centers.'
    )

    # Declare launch arguments for parameters
    resolution_arg = DeclareLaunchArgument('resolution', default_value='0.05', description='Map resolution in meters.')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='odom', description='Fixed frame ID for the map.')
    base_frame_id_arg = DeclareLaunchArgument('base_frame_id', default_value='base_link', description='Base frame ID of the robot.')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time.')
    tf_buffer_duration_arg = DeclareLaunchArgument('tf_buffer_duration', default_value='2.0', description='Duration for TF buffer.')
    sensor_model_max_range_arg = DeclareLaunchArgument('sensor_model_max_range', default_value='5.0', description='Maximum range for sensor model.')
    ground_filter_enable_arg = DeclareLaunchArgument('ground_filter_enable', default_value='False', description='Enable ground filtering.')
    point_cloud_min_z_arg = DeclareLaunchArgument('point_cloud_min_z', default_value='-100.0', description='Minimum Z for point cloud.')
    point_cloud_max_z_arg = DeclareLaunchArgument('point_cloud_max_z', default_value='100.0', description='Maximum Z for point cloud.')
    track_changes_arg = DeclareLaunchArgument('track_changes', default_value='True', description='Enable change tracking.')
    listen_changes_arg = DeclareLaunchArgument('listen_changes', default_value='False', description='Listen for changes.')
    topic_changes_arg = DeclareLaunchArgument('topic_changes', default_value='/mapskit/voxelmap_changes', description='Topic for publishing changes.')
    change_id_frame_arg = DeclareLaunchArgument('change_id_frame', default_value='odom', description='Frame ID for change tracking.')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Path to a parameter file to load. If provided, other parameters are ignored.'
    )

    return LaunchDescription([
        cloud_in_topic_arg,
        voxelmap_full_topic_arg,
        voxelmap_binary_topic_arg,
        voxelmap_centers_topic_arg,
        resolution_arg,
        frame_id_arg,
        base_frame_id_arg,
        use_sim_time_arg,
        tf_buffer_duration_arg,
        sensor_model_max_range_arg,
        ground_filter_enable_arg,
        point_cloud_min_z_arg,
        point_cloud_max_z_arg,
        track_changes_arg,
        listen_changes_arg,
        topic_changes_arg,
        change_id_frame_arg,
        params_file_arg,
        OpaqueFunction(function=launch_setup)
    ])
