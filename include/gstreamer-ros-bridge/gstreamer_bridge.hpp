#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/Image.h>
#include <sensor_msgs/msg/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.h>
#include <std_srvs/SetBool.h>
#include <gst/gst.h>
#include <cstdlib>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

namespace gstreamer_ros_bridge
{

/*
    This node takes an image rostopic and pushes it through a GStreamer pipeline.
*/

class GStreamerBridge : public rclcpp::Node
{
public:
    GStreamerBridge() = default;
    ~GStreamerBridge();

private:
    cv::Mat resize_and_pad(cv::Mat& image, int target_width, int target_height);

    bool set_stream_on_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

    /**
    * @brief Receive from ROS image topic and publish to pipeline to peer.
    * @param msg Image message from ROS.
    * @return void.
    */
    void gs_image_cb(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscriber<sensor_msgs::msg::Image> gs_image_sub_;
    rclcpp::Service<std_srvs::srv::SetBool> set_stream_on_srv_;

    // gstreamer pipeline
    cv::VideoWriter pipeline_;
    cv::Mat frame_;

    // gstreamer param member vars
    int gst_width_, gst_height_, gst_fps_, bitrate_, mtu_;

};

}

#endif // GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
