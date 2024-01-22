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
            # Other necessary parameters
        }],
    )

    # Brightest spot detector node
    brightest_spot_detector_node = Node(
        package='brightest_spot_detector',  # Replace with your package name
        executable='brightest_spot_detector',  # Replace with the name of your executable
        name='brightest_spot_detector',
    )

    return launch.LaunchDescription([
        realsense_camera_node,
        brightest_spot_detector_node
    ])
