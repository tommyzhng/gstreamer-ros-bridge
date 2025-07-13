#include "gstreamer_publisher.hpp"

#include <sstream>
#include <stdexcept>

namespace gstreamer_ros_bridge {

GStreamerPublisher::GStreamerPublisher(const rclcpp::NodeOptions &options, const std::string &out_topic) : Node("gstreamer_publisher_node", options)
{
    img_out_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_topic, 1);
}

GStreamerPublisher::~GStreamerPublisher() {
    gst_in_.release();
}

bool GStreamerPublisher::init(const std::string &pipeline) {
    gst_in_.open(pipeline, cv::CAP_GSTREAMER);
    if (!gst_in_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open gstreamer pipeline!");
        return false;
    }
    return true;
}

bool GStreamerPublisher::init(int port) {
    std::ostringstream oss;
    oss << "udpsrc port=" << port << " ! "
        "application/x-rtp, encoding-name=H264, payload=96 ! "
        "rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! videoscale ! "
        "video/x-raw ! appsink sync=false";
    return init(&oss.str());
}

bool GStreamerPublisher::try_process_frame() {
    cv::Mat frame;
    gst_in_ >> frame;
    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read frame from gstreamer!");
        return false;
    }
    // Timestamp is when the image was received, not captured
    std::msgs::msg::Header header;
    header.stamp = this->now();
    auto image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    img_out_pub_->publish(image_msg);
    return true;
}

} // namespace gstreamer_ros_bridge
RCLCPP_COMPONENTS_REGISTER_NODE(gstreamer_ros_bridge::GStreamerPublisher)
