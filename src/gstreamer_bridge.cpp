#include "gstreamer_bridge.hpp"

namespace gstreamer_ros_bridge
{

GStreamerBridge::GStreamerBridge(const rclcpp::NodeOptions &options) : Node("gstreamer_bridge_node", options)
{
    this->declare_parameter("gst_width", 640);
    this->declare_parameter("gst_height", 480);
    this->declare_parameter("gst_fps", 30);
    this->declare_parameter("bitrate", 1200);
    this->declare_parameter("mtu", 500);

    this->get_parameter("gst_width", gst_width_);
    this->get_parameter("gst_height", gst_height_);
    this->get_parameter("gst_fps", gst_fps_);
    this->get_parameter("bitrate", bitrate_);
    this->get_parameter("mtu", mtu_);

    gs_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_rect", 10, std::bind(&GStreamerBridge::gs_image_cb, this, _1));
    set_stream_on_srv_ = this->create_service<std_srvs::srv::SetBool>("set_stream_on", &GStreamerBridge::set_stream_on_cb);

    start_pipeline();
}

GStreamerBridge::~GStreamerBridge() {
    pipeline_.release();
}

cv::Mat GStreamerBridge::resize_and_pad(cv::Mat &image, int target_width, int target_height) {
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger("rclcpp"), "Input image is empty");
        return cv::Mat();
    }
    if (image.cols == 0 || image.rows == 0) {
        RCLCPP_ERROR(this->get_logger("rclcpp"), "Image has empty rows or columns");
        return cv::Mat();
    }

    float aspect_ratio = float(image.cols) / float(image.rows);
    int new_width = target_width;
    int new_height = target_height;

    if (aspect_ratio > 1.0) {
        new_height = int(target_width / aspect_ratio);
    } else {
        new_width = int(target_height * aspect_ratio);
    }

    // new image with the target dimensions and padding
    cv::Mat padded_image(target_height, target_width, image.type(), cv::Scalar(0, 0, 0));

    int x_offset = (target_width - new_width) / 2;
    int y_offset = (target_height - new_height) / 2;

    // Using ptr for more efficient memory access
    for (int y = 0; y < new_height; ++y) {
        uchar* row_ptr = padded_image.ptr<uchar>(y + y_offset);  // Pointer to row y in the padded image
        for (int x = 0; x < new_width; ++x) {
            int orig_x = int(x * float(image.cols) / new_width);
            int orig_y = int(y * float(image.rows) / new_height);

            orig_x = std::min(orig_x, image.cols - 1);   // Prevent out of bounds
            orig_y = std::min(orig_y, image.rows - 1);

            // Use ptr to access the original image pixels efficiently
            const uchar* orig_row_ptr = image.ptr<uchar>(orig_y);
            row_ptr[x * 3] = orig_row_ptr[orig_x * 3];        // B
            row_ptr[x * 3 + 1] = orig_row_ptr[orig_x * 3 + 1];  // G
            row_ptr[x * 3 + 2] = orig_row_ptr[orig_x * 3 + 2];  // R
        }
    }
    image.release();
    return padded_image;
}

void GStreamerBridge::start_pipeline()
{
    this->declare_parameter("stream_on", true);
    this->declare_parameter("gs_ip", "100.64.0.1");
    this->declare_parameter("gs_port", "5602");

    this->get_parameter("stream_on", stream_on_);
    this->get_parameter("gs_ip", gs_ip);
    this->get_parameter("gs_port", gs_port_);
    
    setenv("GST_DEBUG", "3", 1);
    
    std::ostringstream udp_pipeline;     // gstreamer pipeline for udp to peer
    udp_pipeline 
        << "appsrc ! queue ! videoconvert ! videoscale !"
        << " video/x-raw,width=" << gst_width_ 
        << ",height=" << gst_height_
        << ",framerate=" << gst_fps << "/1 !"
        << " x264enc bitrate=" << bitrate
        << " speed-preset=ultrafast tune=zerolatency !"
        << " rtph264pay mtu=" << mtu << " !"
        << " udpsink host=" << gs_ip
        << " port=" << gs_port
        << " sync=false";

    // start the udp pipeline
    std::string gstreamer_pipeline = udp_pipeline.str();
    pipeline_.open(gstreamer_pipeline, cv::CAP_GSTREAMER, 0, gst_fps, cv::Size(gst_width_, gst_height_), true);
    if (!pipeline_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger("rclcpp"), "Failed to open gstreamer pipeline");
        return;
    }
}

bool GStreamerBridge::set_stream_on_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if (req.data && !pipeline_.isOpened()) {
        resp.success = false;
        resp.message = "pipeline not opened";
    }
    else {
        stream_on_ = req.data;
        resp.success = true;
    }
    RCLCPP_INFO(this->get_logger("rclcpp"), "Turned stream %s", stream_on_ ? "on" : "off");
    return true;
}

void GStreamerBridge::gs_image_cb(const sensor_msgs::msg::Image::SharedPtr &msg)
{
    if (!pipeline_.isOpened()) {
        RCLCPP_ERROR(this->get_logger("rclcpp"), "GStreamer pipeline is not opened, unable to write image");
        return;
    }
    if (!msg) {
        RCLCPP_ERROR(this->get_logger("rclcpp"), "Received an empty image message.");
        return;
    }
    // If stream is turned off, don't write to the pipeline
    if (!stream_on_) {
        return;
    }
    // write to gstreamer pipeline
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::msg::image_encodings::BGR8);
        frame_ = cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger("rclcpp"), "cv_bridge exception: %s", e.what());
    }
    // check if image needs to be resized
    if (frame_.cols != gst_width_ || frame_.rows != gst_height_){
        frame_ = resize_and_pad(frame_, gst_width_, gst_height_);
    }
    pipeline_.write(frame_);
    frame_.release();
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(gstreamer_ros_bridge::GStreamerBridge)
