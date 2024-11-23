# GStreamer ROS Bridge
This package opens a camera stream onto a local ROS topic, allowing for any local node to do processing with minimal latency (such as feeding into an Apriltag detector). It then opens a Gstreamer pipeline to stream any local ROS topic to a remote peer, bypassing the inneficient video networking of ROS and allowing for feedback when monitoring the peer. This GStreamer video can also then be republished as a ROS topic on the peer, allowing for viewing on RQT.


## `gstreamer_bridge_node`

This node initializes an H.264 v4l2 camera stream to a local ROS topic named /camera/image_rect.
It also takes a ROS node specified by the `gst_topic` param and writes it to a custom GStreamer pipeline to the peer.

Params:
- `camera_location`: Camera to start the stream to the local ROS topic. Default /dev/video0
- `camera_width`, `camera_height`, `camera_fps`: Specifies the input stream parameters to the locally published ROS topic. This way higher resolution can be published to local processing nodes like April tags or CV. Default 1280, 720, 60.
- `custom_pipline`: Fill in if a custom local camera pipeline is required. e.g IMX219 likes nvarguscamsrc on Jetson Orins.
- `bitrate`, `mtu`: Specifies the bitrate and MTU of the GStreamer pipeline to the peer. Slower networks usually require lower bitrates so the network can keep up.
- `gst_width`, `gst_height`, `gst_fps`: The GStreamer pipeline resolution and framerate. This allows decreased bandwidth after processing the image locally by sending a downscaled image to the peer for feedback purposes. Default 640, 360, 15.
- `ip`, `port`: The Peer IP and port of the local stream. Default 100.64.0.3, 5602.


## `gstreamer_publisher_node`

This node receives data from a GStreamer pipeline and publishes it to a ROS topic as a `sensor_msgs/Image` message.
By default it expects H.264 encoded video sent over UDP. This is usually run on the 'peer' computer to connect to the bridge node.

* Important note: this must be run before the `gstreamer_bridge_node` to make sure the camera pops up every time in rqt.

Params:
- `topic_name`: Name of the topic to publish the images to. Default `/gstreamer_received`.
- `port`: UDP port to receive the stream from. This should be the same as the port specified on the bridge node. Default 5602.
- `pipeline`: If specified, this will be used as the gstreamer pipeline. It will completely replace the default pipeline (so `port` isn't used anymore), allowing you to receive the stream however you want. By default this is empty, so the normal H.264 over UDP pipeline is used.
