import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# --- Configuration ---
this_dir = os.path.dirname(os.path.abspath(__file__))
TF_BROADCASTER_SCRIPT = os.path.join(this_dir, 'tf_broadcaster.py')


def generate_launch_description():
    
    # 1. ROS Bridge Launch Inclusion
    rosbridge_launch_dir = os.path.join(
        get_package_share_directory('rosbridge_server'), 
        'launch'
    )
    
    ros_bridge_server = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource( 
            os.path.join(rosbridge_launch_dir, 'rosbridge_websocket_launch.xml')
        )
    )

    # 2. Point Cloud Publisher Node
    pointCloudPublisher = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name='depth_to_pointcloud',
        remappings=[
            ('image_rect', '/arkit/Image/depth_raw'),
            ('camera_info', '/arkit/depth/camera_info'),
            ('points', '/arkit/point_cloud'),
        ],
        output='screen'
    )

    # 3. Static Transform Publisher Node
    static_cameraframe_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_depth_tf_publisher',
        arguments=['0', '0', '0', '-1.57', '0', '0', 'base_link', 'camera_depth_frame'],
        output='screen'
    )

    # 4. Custom TF Broadcaster Node (Package field removed for global executable)
    custom_tf_broadcaster_node = Node(
        executable='python3', 
        name='custom_dynamic_tf_broadcaster',
        arguments=[TF_BROADCASTER_SCRIPT],
        # The package argument has been removed here to resolve the libexec error.
        output='screen'
    )

    return LaunchDescription([
        ros_bridge_server,
        pointCloudPublisher,
        static_cameraframe_node,
        custom_tf_broadcaster_node
    ])