# gstreamer-ros-bridge
Bridges ROS image topics to gstreamer piplines and vice versa

## `gstreamer_publisher_node`

This node receives data from a gstreamer pipeline and publishes it to a ROS topic as a `sensor_msgs/Image` message.
By default it expects H.264 encoded video sent over UDP.

Params:
- `topic_name`: Name of the topic to publish the images to. Default `/gstreamer_received`.
- `port`: UDP port to receive the stream from. Default 5602.
- `pipeline`: If specified, this will be used as the gstreamer pipeline. It will completely replace the default pipeline (so `port` isn't used anymore), allowing you to receive the stream however you want. By default this is empty, so the normal H.264 over UDP pipeline is used.
