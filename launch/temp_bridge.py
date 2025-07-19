from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

gst_width = 640
gst_height = 480
gst_fps = 30

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
                    plugin='gstreamer_ros_bridge::GStreamerBridge',
                    name='gstreamer_bridge_node',
                    parameters=[{
                        'stream_on': True,
                        'gs_ip': "100.64.9.7",
                        'gs_port': "5602",
                        'gst_width': gst_width,
                        'gst_height': gst_height,
                        'gst_fps': gst_fps,
                        'bitrate': 1200,
                        'mtu': 500,
                        'cam_topic': "camera/image_raw"
                    }]
                )
            ],
            output='screen',
        )
    ])


