#include "gstreamer_cam.hpp"

namespace gstreamer_ros_bridge
{

GStreamerCam::GStreamerCam()
: Node("gstreamer_cam_node"),
{
    // get params
    this->declare_parameter("camera_location", "v4l2src device=/dev/video0");
    this->declare_parameter("camera_format", "BGR");
    this->declare_parameter("camera_topic", "camera/image_raw");
    this->declare_parameter("camera_width", 640);
    this->declare_parameter("camera_height", 480);
    this->declare_parameter("camera_fps", 30);

    this->get_parameter("camera_location", camera_location_);
    this->get_parameter("camera_format", camera_format_);
    this->get_parameter("camera_topic", camera_topic_);
    this->get_parameter("camera_width", camera_width_);
    this->get_parameter("camera_height", camera_height_);
    this->get_parameter("camera_fps", camera_fps_);

    // initialize image transport
    it_ = image_transport::ImageTransport(this->shared_from_this());
    ros_image_pub_ = it_.advertise(camera_topic_, 1);
    ros_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 1);

    // initialize gstreamer
    initialize_gstreamer();

    // set camera info parameters
    set_cam_info();
}

GStreamerCam::~GStreamerCam()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(appsink_));
        gst_object_unref(GST_OBJECT(pipeline_));
    }
}

void GStreamerCam::set_cam_info()
{
    this->declare_parameter("calibration/model", "plumb_bob");
    this->declare_parameter("calibration/D", std::vector<double>{});
    this->declare_parameter("calibration/K", std::vector<double>{});
    this->declare_parameter("calibration/R", std::vector<double>{});
    this->declare_parameter("calibration/P", std::vector<double>{});

    this->get_parameter("calibration/model", camera_info_.distortion_model);
    this->get_parameter("calibration/D", camera_info_.d);
    this->get_parameter("calibration/K", camera_info_.k);
    this->get_parameter("calibration/R", camera_info_.r);
    this->get_parameter("calibration/P", camera_info_.p);

    camera_info_.header.frame_id = camera_topic_;
    camera_info_.height = camera_height_;
    camera_info_.width = camera_width_;
    std::copy(K.begin(), K.end(), camera_info_.k.begin());
    std::copy(R.begin(), R.end(), camera_info_.r.begin());
    std::copy(P.begin(), P.end(), camera_info_.p.begin());
}

void GStreamerCam::initialize_gstreamer()
{
    std::ostringstream cam_pipeline;
    cam_pipeline
        << "v4l2src device=" << camera_location_ << " !"
        << " videoconvert ! videoscale ! video/x-raw,format=" << camera_format_
        << ",width=" << camera_width_
        << ",height=" << camera_height_
        << ",framerate=" << camera_fps_ << "/1 !"
        << " appsink name=sink sync=false drop=true max-buffers=1";

    std::string pipeline_str = cam_pipeline.str();

    this->declare_parameter("custom_pipeline", pipeline_str);
    this->get_parameter("custom_pipeline", pipeline_str);
    const gchar *pipeline_desc = pipeline_str.c_str();

    pipeline_ = gst_parse_launch(pipeline_desc, NULL);
    NODELET_INFO("GStreamer pipeline: %s", pipeline_desc);

    if (!pipeline_) {
        NODELET_ERROR("Pipeline was not opened.");
        return;
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

    if (!appsink_) {
        NODELET_ERROR("Failed to get appsink element from the pipeline.");
        gst_object_unref(pipeline_);
        return;
    }

    g_object_set(appsink_, "emit-signals", FALSE, "sync", FALSE, "drop", TRUE, "max-buffers", 1, NULL);    
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
}

void GStreamerCam::ConvertImage(GstMapInfo &map_info)
{
    if (g_strcmp0(format_, "NV12") == 0) {
        // NV12: YUV 4:2:0, semi-planar
        cv::Mat nv12(camera_height_ + camera_height_ / 2, camera_width_, CV_8UC1, (void*)map_info.data);
        cv::cvtColor(nv12, frame_, cv::COLOR_YUV2BGR_NV12);
    } else if (g_strcmp0(format_, "YUY2") == 0) {
        // YUY2: YUV 4:2:2
        cv::Mat yuy2(camera_height_, camera_width_, CV_8UC2, (void*)map_info.data);
        cv::cvtColor(yuy2, frame_, cv::COLOR_YUV2BGR_YUY2);
    } else if (g_strcmp0(format_, "RGB") == 0 || g_strcmp0(format_, "BGR") == 0) {
        // RGB/BGR formats (just clone it bc its supported)
        int channels = 3;
        cv::Mat img(camera_height_, camera_width_, CV_8UC3, (void*)map_info.data);
        frame_ = img.clone();
    } else if (g_strcmp0(format_, "GRAY8") == 0) {
        // Grayscale format
        cv::Mat gray(camera_height_, camera_width_, CV_8UC1, (void*)map_info.data);
        cv::cvtColor(gray, frame_, cv::COLOR_GRAY2BGR);
    } else {
        NODELET_WARN("Unsupported camera format: %s", format_);
        return;
    }
}

void GStreamerCam::PubCameraImage()
{
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), GST_SECOND);
    if (!sample) {
        NODELET_WARN("Failed to get GstSample from appsink");
        return;
    }
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        NODELET_WARN("Failed to get GstBuffer from sample");
        gst_sample_unref(sample);
        return;
    }
    // map the buffer to access the data
    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
        NODELET_WARN("Failed to map GstBuffer");
        gst_sample_unref(sample);
        return;
    }
    // check if data is empty
    if (!map_info.data) {
        NODELET_WARN("Buffer data is null");
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
        return;
    }
    // check format once to avoid multiple calls
    if (!format_) {
        GstCaps *caps = gst_sample_get_caps(sample);
        if (!caps) {
            NODELET_WARN("Failed to get GstCaps from sample");
            gst_sample_unref(sample);
            return;
        }
        // get the format from the caps
        GstStructure *s = gst_caps_get_structure(caps, 0);
        format_ = gst_structure_get_string(s, "format");
        NODELET_INFO("Camera format: %s", format_);
        if (!format_) {
            NODELET_WARN("Failed to get format from caps");
            gst_sample_unref(sample);
            return;
        }
    }
    // convert to readable format by cv_bridge
    ConvertImage(map_info);
    try {
        sensor_msgs::msg::Image msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_).toImageMsg();
        // publish image
        msg.header.stamp = this->now();
        ros_image_pub_.publish(msg);
        // publish camera info
        camera_info_.header.stamp = this->now();
        ros_camera_info_pub_->publish(camera_info_);
    } catch (const std::exception &e) {
        NODELET_ERROR("Failed to convert frame to ROS Image message: %s", e.what());
    }
    // unmap and release the buffer
    gst_buffer_unmap(buffer, &map_info);
    gst_sample_unref(sample);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(gstreamer_ros_bridge::GStreamerCam)