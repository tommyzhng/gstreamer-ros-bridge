#include "gstreamer_bridge.hpp"

GStreamerRosBridge::GStreamerRosBridge(ros::NodeHandle &nh)
{
    std::string gs_ip = "100.64.0.1";
    nh.getParam("ip", gs_ip);

    std::string gs_port = "5602";
    nh.getParam("port", gs_port);

    int gst_fps, bitrate, mtu;
    nh.getParam("gst_width", gst_width_);
    nh.getParam("gst_height", gst_height_);
    nh.getParam("gst_fps", gst_fps);
    nh.getParam("bitrate", bitrate);
    nh.getParam("mtu", mtu);

    setenv("GST_DEBUG", "3", 1); // somehow this helps with querying video position

    std::ostringstream udpPipeline;     // gstreamer pipeline for udp to peer
    udpPipeline 
        << "appsrc ! videoconvert ! videoscale !"
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
    nh.getParam("gst_topic", gst_topic);

    // start the udp pipeline
    std::string gstreamer_pipeline = udpPipeline.str();
    pipeline_.open(gstreamer_pipeline, cv::CAP_GSTREAMER, 0, gst_fps, cv::Size(gst_width_, gst_height_), true);
    if (!pipeline_.isOpened())
    {
        ROS_ERROR("Failed to open gstreamer pipeline");
        return;
    }

    // ros subscribers and publishers
    gsImageSub_ = nh.subscribe(gst_topic, 1, &GStreamerRosBridge::GsImageCallback, this);
}

GStreamerRosBridge::~GStreamerRosBridge()
{
    pipeline_.release();
}

void GStreamerRosBridge::GsImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if (!pipeline_.isOpened()) {
        ROS_ERROR("GStreamer pipeline is not opened, unable to write image");
        return;
    }
    // write to gstreamer pipeline
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat image = cv_ptr->image;
        //cv::resize(image, image, cv::Size(gst_width_, gst_height_));
        image.convertTo(image, CV_8UC3);
        pipeline_.write(image);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}
