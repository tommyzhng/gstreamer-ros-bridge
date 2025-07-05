from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

camera_location = "/dev/video0"  
camera_format = "BGR"
camera_width = 640
camera_height = 480
camera_fps = 30

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='gstreamer_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='gstreamer_ros_bridge',
                    plugin='gstreamer_ros_bridge::GStreamerCam',
                    name='gstreamer_cam_node',
                    parameters=[{
                        'camera_location': camera_location,
                        'camera_format': camera_format,
                        'camera_topic': 'camera/image_raw',
                        'camera_width': camera_width,
                        'camera_height': camera_height,
                        'camera_fps': camera_fps,
                        'custom_pipeline': f'v4l2src device={camera_location} ! videoconvert ! videoscale ! video/x-raw,format={camera_format},width={camera_width},height={camera_height},framerate={camera_fps}/1 ! appsink name=sink sync=false drop=true max-buffers=1',
                        'calibration/model': 'plumb_bob',
                        'calibration/D': [0.0, 0.0, 0.0, 0.0, 0.0],
                        'calibration/K': [640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0],
                        'calibration/R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                        'calibration/P': [640.0, 0.0, 320.0, 0.0, 0.0, 480.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                    }]
                )
            ],
            output='screen',
        )
    ])
