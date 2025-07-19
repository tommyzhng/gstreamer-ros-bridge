#include "gstreamer_publisher.hpp"

#include <sstream>
#include <stdexcept>

namespace gstreamer_ros_bridge {

GStreamerPublisher::GStreamerPublisher(const rclcpp::NodeOptions &options, const std::string &out_topic) : Node("gstreamer_publisher_node", options)
{
    // initialize GStreamer
    gst_init(nullptr, nullptr);
    img_out_pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_topic, 1);
    this->declare_parameter("port", 5602);
    this->get_parameter("port", port_);
    
    init(port_);
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / 30),  // 30 FPS
        std::bind(&GStreamerPublisher::try_process_frame, this));
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
    return init(oss.str());
}

bool GStreamerPublisher::try_process_frame() {
    cv::Mat frame;
    gst_in_ >> frame;
    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read frame from gstreamer!");
        return false;
    }
    // Timestamp is when the image was received, not captured
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera";
        
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
    cv_bridge::CvImage cv_image(header, "bgr8", frame);
    cv_image.toImageMsg(*image_msg);
    img_out_pub_->publish(*image_msg);

    return true;
}

} // namespace gstreamer_ros_bridge
RCLCPP_COMPONENTS_REGISTER_NODE(gstreamer_ros_bridge::GStreamerPublisher)
