from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

port = 5602

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='gstreamer_publisher_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='gstreamer_ros_bridge',
                    plugin='gstreamer_ros_bridge::GStreamerPublisher',
                    name='gstreamer_publisher_node',
                    parameters=[{
                        'port': port,
                    }]
                )
            ],
            output='screen',
        )
    ])        