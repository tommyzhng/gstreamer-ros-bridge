#include "gstreamer_bridge.hpp"
#include <chrono>
// function to replace opencv resize since it seg faults in version 4.2 
cv::Mat resizeAndPad(cv::Mat &image, int target_width, int target_height) {
    if (image.empty()) {
        ROS_ERROR("Input image is empty");
        return cv::Mat();
    }
    if (image.cols == 0 || image.rows == 0) {
        ROS_ERROR("Image has empty rows or columns");
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

            // Bounds checking: Ensure we are not accessing out-of-bounds pixels
            orig_x = std::min(orig_x, image.cols - 1);
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
    ros_rate_ = ros::Rate(gst_fps);  // update ros rate

    setenv("GST_DEBUG", "3", 1); // somehow this helps with querying video position

    std::ostringstream udpPipeline;     // gstreamer pipeline for udp to peer
    udpPipeline 
        << "appsrc ! videoconvert ! videoscale !"
        << " video/x-raw,width=" << gst_width_ 
        << ",height=" << gst_height_
        << ",framerate=" << gst_fps << "/1 !"
        << " x264enc bitrate=" << bitrate
        << " speed-preset=ultrafast tune=zerolatency byte-stream=true key-int-max=15 intra-refresh=true !"
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

    // local image subscriber
    gsImageSub_ = nh.subscribe(gst_topic, 1, &GStreamerRosBridge::GsImageCallback, this, ros::TransportHints().tcpNoDelay());
}

GStreamerRosBridge::~GStreamerRosBridge()
{
    pipeline_.release();
}

void GStreamerRosBridge::GsImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
{
    if (!pipeline_.isOpened()) {
        ROS_ERROR("GStreamer pipeline is not opened, unable to write image");
        return;
    }
    if (!msg) {
        ROS_ERROR("Received an empty image message.");
        return;
    }   
    // write to gstreamer pipeline
      try {
        // Decode the compressed image into an OpenCV Mat
        auto start = std::chrono::high_resolution_clock::now();

        cv::Mat compressedImage = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        auto decode_end = std::chrono::high_resolution_clock::now();

        if (compressedImage.empty()) {
            ROS_ERROR("Failed to decode the compressed image.");
            return;
        }

        // If needed, resize the image
        if (compressedImage.cols != gst_width_ || compressedImage.rows != gst_height_) {
            compressedImage = resizeAndPad(compressedImage, gst_width_, gst_height_);
        }

        // Write to the GStreamer pipeline
        pipeline_.write(compressedImage);
    } catch (const std::exception &e) {
        ROS_ERROR("Exception while processing compressed image: %s", e.what());
    }
}

