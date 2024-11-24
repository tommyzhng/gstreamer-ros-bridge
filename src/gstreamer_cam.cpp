#include "gstreamer_cam.hpp"

GStreamerCam::GStreamerCam(ros::NodeHandle &nh)
{
    nh.getParam("camera_location", camera_location_);
    nh.getParam("camera_format", camera_format_);
    nh.getParam("camera_width", camera_width_);
    nh.getParam("camera_height", camera_height_);
    nh.getParam("camera_fps", camera_fps_);
    nh.getParam("camera_topic", camera_topic_);

    rosImagePub_ = nh.advertise<sensor_msgs::Image>("/camera/image_rect", 1);
    rosCameraInfoPub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
    
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
        << " appsink name=sink sync=false";

    std::string pipeline_str = camPipeline.str();
    const gchar *pipeline_desc = pipeline_str.c_str();
    pipeline_ = gst_parse_launch(pipeline_desc, NULL);

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
    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));

    if (sample) {
        //get the buffer from the sample
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            ROS_WARN("Failed to get GstBuffer from sample.");
            gst_sample_unref(sample);
            return;
        }

        // map the buffer to access the data
        GstMapInfo map_info;
        if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
            ROS_WARN("Failed to map GstBuffer.");
            gst_sample_unref(sample);
            return;
        }

        // Validate the size of the mapped data
        if (map_info.size != 1280 * 720 * 3) {
            ROS_WARN("Unexpected buffer size: %zu. Expected: %d.", map_info.size, 1280 * 720 * 3);
            gst_buffer_unmap(buffer, &map_info);
            gst_sample_unref(sample);
            return;
        }

        cv::Mat frame(camera_height_, camera_width_, CV_8UC3, (void*)map_info.data);

        try {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
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

void GStreamerCam::Update()
{
    PubCameraImage();
}
