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
            'enable_infra1': True,  # Enable the first IR camera
            'enable_infra2': False, # Disable the second IR camera if not needed
            'enable_color': False,  # Disable other streams if not needed
            'enable_depth': False,
            'depth_module.emitter_enabled': 0,
        }],
    )

    # Brightest spot detector node
    bright_spot_detector_node = Node(
        package='bright_spot_detector',  # Replace with your package name
        executable='bright_spot_detector',  # Replace with the name of your executable
        name='bright_spot_detector',
        parameters=[{
            'brightness_threshold': 250.0,
            'area_threshold': 10.0,
        }]
    )

    return launch.LaunchDescription([
        realsense_camera_node,
        bright_spot_detector_node
    ])
