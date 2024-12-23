#include "gstreamer_cam.hpp"

GStreamerCam::GStreamerCam(ros::NodeHandle &nh)
{
    nh_ = nh;
    nh_.getParam("camera_location", camera_location_);
    nh_.getParam("camera_format", camera_format_);
    nh_.getParam("camera_width", camera_width_);
    nh_.getParam("camera_height", camera_height_);
    nh_.getParam("camera_fps", camera_fps_);
    nh_.getParam("camera_topic", camera_topic_);

    rosImagePub_ = nh_.advertise<sensor_msgs::Image>("/camera/image_rect", 1);
    rosCameraInfoPub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
    
    // init the gstreamer pipeline
    InitializeGStreamer(); 
    SetCameraParams(); // set distortions and camera info (not fully implemented yet)
}

GStreamerCam::~GStreamerCam()
{
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(appsink_);
    gst_object_unref(pipeline_);
}

void GStreamerCam::SetCameraParams()
{   
    cameraInfo_.header.frame_id = "camera";
    cameraInfo_.height = 480;
    cameraInfo_.width = 640;
    cameraInfo_.distortion_model = "plumb_bob";
    cameraInfo_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cameraInfo_.K = {640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0};
    cameraInfo_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cameraInfo_.P = {640.0, 0.0, 320.0, 0.0, 0.0, 480.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
}

void GStreamerCam::InitializeGStreamer()
{    
    std::ostringstream camPipeline;
    camPipeline
        << "v4l2src device=" << camera_location_ << " !"
        << " videoconvert ! videoscale ! video/x-raw,format=" << camera_format_
        << ",width=" << camera_width_
        << ",height=" << camera_height_
        << ",framerate=" << camera_fps_ << "/1 !"
        << " appsink name=sink sync=false drop=true max-buffers=1";

    std::string pipeline_str = camPipeline.str();

    nh_.getParam("custom_pipeline", pipeline_str);  // load if there is a custom pipeline
    const gchar *pipeline_desc = pipeline_str.c_str();

    
    pipeline_ = gst_parse_launch(pipeline_desc, NULL);
    ROS_INFO("GStreamer pipeline: %s", pipeline_desc);

    if (!pipeline_) {
        ROS_ERROR("Pipeline was not opened.");
        return;
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

    if (!appsink_) {
        ROS_ERROR("Failed to get appsink element from the pipeline.");
        gst_object_unref(pipeline_);
        return;
    }

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
}

void GStreamerCam::PubCameraImage()
{
    GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), GST_SECOND);
    if (sample) {
        //get the buffer from the sample
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            ROS_WARN("Failed to get GstBuffer from sample");
            gst_sample_unref(sample);
            return;
        }
        // map the buffer to access the data
        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
            ROS_WARN("Failed to map GstBuffer");
            gst_sample_unref(sample);
            return;
        }
        // check format
        if (!format_) {
            GstCaps *caps = gst_sample_get_caps(sample);
            if (!caps) {
                ROS_WARN("Failed to get GstCaps from sample");
                gst_sample_unref(sample);
                return;
            }
            // get the format from the caps
            GstStructure *s = gst_caps_get_structure(caps, 0);
            format_ = gst_structure_get_string(s, "format");
            ROS_INFO("Camera format: %s", format_);
            if (!format_) {
                ROS_WARN("Failed to get format from caps");
                gst_sample_unref(sample);
                return;
            }
        }

        // check if empty
        if (!map_info.data) {
            ROS_WARN("Buffer data is null");
            gst_buffer_unmap(buffer, &map_info);
            gst_sample_unref(sample);
            return;
        }
        // convert to readable format by cv_bridge
        ConvertImage(map_info);
        try {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
            ros::Time current_time = ros::Time::now();
            
            msg->header.stamp = current_time;
            rosImagePub_.publish(msg);

            cameraInfo_.header.stamp = current_time;
            rosCameraInfoPub_.publish(cameraInfo_);
        } catch (const std::exception &e) {
            ROS_ERROR("Failed to convert frame to ROS Image message: %s", e.what());
        }
        // Unmap and release the buffer
        gst_buffer_unmap(buffer, &map_info);
        gst_sample_unref(sample);
    } else {
        ROS_WARN("Failed to get frame from appsink");
    }
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
        ROS_WARN("Unsupported camera format: %s", format_);
        return;
    }
}

void GStreamerCam::Update()
{
    PubCameraImage();
}
