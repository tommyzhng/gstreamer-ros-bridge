#include "gstreamer_publisher.hpp"

#include <sstream>
#include <stdexcept>

#include "cv_bridge/cv_bridge.h"

GstreamerPublisher::GstreamerPublisher(ros::NodeHandle nh, const std::string &out_topic) : nh_(nh) {
    imgOutPub_ = nh_.advertise<sensor_msgs::Image>(out_topic, 1);
}

GstreamerPublisher::~GstreamerPublisher() {
    gstIn_.release();
}

bool GstreamerPublisher::init(const std::string &pipeline) {
    gstIn_.open(pipeline, cv::CAP_GSTREAMER);
    if (!gstIn_.isOpened()) {
        ROS_ERROR("Failed to open gstreamer pipeline!");
        return false;
    }
    return true;
}

bool GstreamerPublisher::init(int port) {
    std::ostringstream oss;
    oss << "udpsrc port=" << port << " ! "
        "application/x-rtp, encoding-name=H264, payload=96 ! "
        "rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! videoscale ! "
        "video/x-raw ! appsink sync=false";
    return init(oss.str());
}

bool GstreamerPublisher::tryProcessFrame() {
    cv::Mat frame;
    gstIn_ >> frame;
    if (frame.empty()) {
        ROS_ERROR("Failed to read frame from gstreamer!");
        return false;
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    // Timestamp is when the image was received, not captured
    msg->header.stamp = ros::Time::now();
    imgOutPub_.publish(msg);
    return true;
}
