# need to fix docs for ros2

# GStreamer ROS Bridge

This package opens a camera stream onto a local ROS topic, allowing for any local node to do processing with minimal latency (such as feeding into an Apriltag detector). It then opens a Gstreamer pipeline to stream any local ROS topic to a remote peer, bypassing the inneficient video networking of ROS and allowing for feedback when monitoring the peer. This GStreamer video can also then be republished as a ROS topic on the peer, allowing for viewing on RQT.

## Launch Files

### `"camera".launch`
**Arguments (Camera):** \
This node initializes a camera stream (v4l2, nvarguscamera or libcamera) and publishes to a local ROS topic named /camera/image_rect.

- `camera_width`, `camera_height`, `camera_fps`: Specifies the input stream parameters to the locally published ROS topic.
- `camera_format`: Specifies the format of the camera.
- `camera_location`: Camera to start the stream to the local ROS topic. E.g. /dev/video0
- `camera_topic`: Specifies what topic the camera node publishes to. 
- `custom_pipline`: Fill in the pipeline that works with your camera. Test this with CLI and see if it works with a UDP or file sink. Then convert the output to an appsink as seen in the examples.

**Arguments (Bridge):** \
This node takes a ROS topic specified by the `gst_topic` param and writes it to a custom GStreamer pipeline to the peer IP.
- `gst_width`, `gst_height`, `gst_fps`: The GStreamer pipeline resolution and framerate. This allows decreased bandwidth after processing the image locally by sending a downscaled image to the peer for feedback purposes.
- `gst_topic`: The topic that the pipeline should take images from. Default /camera/image_rect
- `ip`, `port`: The Peer IP and port of the local stream.
- `bitrate`, `mtu`: Specifies the bitrate and MTU of the GStreamer pipeline to the peer. Slower networks usually require lower bitrates so the network can keep up.
- `stream_on`: Whether the stream is turned on initially. The stream can be turned on or off after initialization by calling the service `~set_stream_on` of type `std_msgs/SetBool`.

### `gstreamer_publisher.launch`

This node receives data from a GStreamer pipeline and publishes it to a ROS topic as a `sensor_msgs/Image` message.
By default it expects H.264 encoded video sent over UDP. This is usually run on the 'peer' computer to connect to the bridge node.

* Note: this should be run before the `gstreamer_bridge_node` for faster response (not crucial, but recommended).

Params:
- `topic_name`: Name of the topic to publish the images to. Default `/gstreamer_received`.
- `port`: UDP port to receive the stream from. This should be the same as the port specified on the bridge node. Default 5602.
- `pipeline`: If specified, this will be used as the gstreamer pipeline. It will completely replace the default pipeline (so `port` isn't used anymore), allowing you to receive the stream however you want. By default this is empty, so the normal H.264 over UDP pipeline is used.
