#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/Image.h>
#include <sensor_msgs/msg/camera_info.h>
#include <std_srvs/SetBool.h>
#include <gst/gst.h>
#include <cstdlib>
#include <functional>

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
    virtual void onInit() override;

    cv::Mat resizeAndPad(cv::Mat& image, int target_width, int target_height);

    bool SetStreamOnCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

    /**
    * @brief Receive from ROS image topic and publish to pipeline to peer.
    * @param msg Image message from ROS.
    * @return void.
    */
    void GSImageCallback(const sensor_msgs::msg::ImageConstPtr& msg);

    rclcpp::Subscriber<sensor_msgs::msg::Image> gsImageSub_;
    rclcpp::Service<std_srvs::srv::SetBool> set_stream_on_srv_;

    // gstreamer pipeline
    cv::VideoWriter pipeline_;
    cv::Mat frame_;

    // gstreamer param member vars
    int gst_width_{640}, gst_height_{480};

    bool stream_on_ = true;

};

}

#endif // GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP