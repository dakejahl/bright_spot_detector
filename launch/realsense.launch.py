import launch
from launch_ros.actions import Node

def generate_launch_description():
    # RealSense camera node with IR stream enabled
    realsense_camera_node = Node(
        package='realsense2_camera',
        namespace='camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': False,
            'enable_color': True,
            'enable_depth': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.profile': '640x480x15',
        }],
    )

    # Brightest spot detector node
    pc_overlay_node = Node(
        package='pc_overlay',
        executable='pc_overlay',
        name='pc_overlay',
    )

    return launch.LaunchDescription([
        realsense_camera_node,
        pc_overlay_node
    ])
