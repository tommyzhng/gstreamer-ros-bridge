#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_PUBLISHER_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_PUBLISHER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "cv_bridge/cv_bridge.h"
#include <gst/gst.h>

namespace gstreamer_ros_bridge {

class GStreamerPublisher : public rclcpp::Node
{
public:
    GStreamerPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), const std::string &out_topic = "/gstreamer/received");
    ~GStreamerPublisher();

    GStreamerPublisher(const GStreamerPublisher&) = delete;
    GStreamerPublisher& operator=(const GStreamerPublisher&) = delete;

    /**
     * @brief Initialize the pipeline.
     * 
     * @param port UDP port that the stream will be read from.
     * @return True if initialization was successful.
     */
    bool init(const std::string &pipeline);
    bool init(int port);
    /**
     * @brief Try to receive and publish a single frame to the ROS topic.
     * 
     * @return Whether the frame was processed properly.
     */
     bool try_process_frame();

private:
    
    cv::VideoCapture gst_in_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_out_pub_;
    
    rclcpp::TimerBase::SharedPtr update_timer_;
    int port_; // param to connect to the port from the other device
};

} // namespace gstreamer_ros_bridge
#endif
