#include "gstreamer_bridge.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gstreamer_ros_bridge::GStreamerRosBridge, nodelet::Nodelet)

namespace gstreamer_ros_bridge
{

cv::Mat GStreamerRosBridge::resizeAndPad(cv::Mat &image, int target_width, int target_height) {
    if (image.empty()) {
        NODELET_ERROR("Input image is empty");
        return cv::Mat();
    }
    if (image.cols == 0 || image.rows == 0) {
        NODELET_ERROR("Image has empty rows or columns");
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

GStreamerRosBridge::~GStreamerRosBridge()
{
    pipeline_.release();
}

void GStreamerRosBridge::onInit()
{
    nh_ = getPrivateNodeHandle();

    nh_.getParam("stream_on", stream_on_);

    std::string gs_ip = "100.64.0.1";
    nh_.getParam("ip", gs_ip);
    std::string gs_port = "5602";
    nh_.getParam("port", gs_port);

    int gst_fps = 30, bitrate = 1200, mtu = 500;
    nh_.getParam("gst_width", gst_width_);
    nh_.getParam("gst_height", gst_height_);
    nh_.getParam("gst_fps", gst_fps);
    nh_.getParam("bitrate", bitrate);
    nh_.getParam("mtu", mtu);

    setenv("GST_DEBUG", "3", 1);
    
    std::ostringstream udpPipeline;     // gstreamer pipeline for udp to peer
    udpPipeline 
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
    
    std::string gst_topic = "/camera/image_rect";
    nh_.getParam("gst_topic", gst_topic);

    // start the udp pipeline
    std::string gstreamer_pipeline = udpPipeline.str();
    pipeline_.open(gstreamer_pipeline, cv::CAP_GSTREAMER, 0, gst_fps, cv::Size(gst_width_, gst_height_), true);
    if (!pipeline_.isOpened())
    {
        NODELET_ERROR("Failed to open gstreamer pipeline");
        return;
    }

    gsImageSub_ = nh_.subscribe(gst_topic, 1, &GStreamerRosBridge::GsImageCallback, this, ros::TransportHints().tcpNoDelay());

    set_stream_on_srv_ = nh_.advertiseService("set_stream_on", &GStreamerRosBridge::SetStreamOnCallback, this);
}

bool GStreamerRosBridge::SetStreamOnCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    if (req.data && !pipeline_.isOpened()) {
        resp.success = false;
        resp.message = "pipeline not opened";
    }
    else {
        stream_on_ = req.data;
        resp.success = true;
    }
    NODELET_INFO("Turned stream %s", stream_on_ ? "on" : "off");
    return true;
}

void GStreamerRosBridge::GsImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if (!pipeline_.isOpened()) {
        NODELET_ERROR("GStreamer pipeline is not opened, unable to write image");
        return;
    }
    if (!msg) {
        NODELET_ERROR("Received an empty image message.");
        return;
    }
    // If stream is turned off, don't write to the pipeline
    if (!stream_on_) {
        return;
    }
    // write to gstreamer pipeline
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        frame_ = cv_ptr->image;
    } catch (cv_bridge::Exception &e) {
        NODELET_ERROR("cv_bridge exception: %s", e.what());
    }
    // check if image needs to be resized
    if (frame_.cols != gst_width_ || frame_.rows != gst_height_){
        frame_ = resizeAndPad(frame_, gst_width_, gst_height_);
    }
    pipeline_.write(frame_);
    frame_.release();
}

}
