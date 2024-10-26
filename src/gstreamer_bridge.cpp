#include "gstreamer_bridge.hpp"

GstreamerRosBridge::GstreamerRosBridge(ros::NodeHandle &nh) : cap_("/dev/video0", cv::CAP_V4L2)
{
    std::string gstreamer_pipeline = 
            "appsrc ! videoconvert ! videoscale ! "
            "video/x-raw,width=640,height=480,framerate=30/1 ! "
            "x264enc bitrate=1200 speed-preset=ultrafast tune=zerolatency ! "
            "rtph264pay mtu=900 ! udpsink host=100.64.0.3 port=5602 sync=false";
        
    pipeline_.open(gstreamer_pipeline, cv::CAP_GSTREAMER, 0, 30.0, cv::Size(640, 480), true);
    if (!pipeline_.isOpened())
    {
        ROS_ERROR("Failed to open gstreamer pipeline");
        return;
    }

    gsImageSub_ = nh.subscribe("/tag_detections_image", 1, &GstreamerRosBridge::gsImageCallback, this);
    rosCameraInfoPub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);
    rosImagePub_ = nh.advertise<sensor_msgs::Image>("/camera/image_rect", 1);

    // initialize camera info
    cameraInfo_.header.frame_id = "camera";
    cameraInfo_.height = 480;
    cameraInfo_.width = 640;
    cameraInfo_.distortion_model = "plumb_bob";
    cameraInfo_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cameraInfo_.K = {640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0};
    cameraInfo_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cameraInfo_.P = {640.0, 0.0, 320.0, 0.0, 0.0, 480.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};

    startImageCapture();
}

GstreamerRosBridge::~GstreamerRosBridge()
{
    cap_.release();
    pipeline_.release();
}

void GstreamerRosBridge::startImageCapture()
{
    cv::Mat frame;
    ros::Rate rate(30);

    while (nh_.ok())
    {
        cap_ >> frame;
        if (frame.empty())
        {
            ROS_ERROR("No image captured (its gg)");
            return;
        }
        // store current time in header
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();
        cameraInfo_.header.stamp = ros::Time::now();

        rosImagePub_.publish(msg);
        rosCameraInfoPub_.publish(cameraInfo_);
        ros::spinOnce();
        rate.sleep();
    }
}

void GstreamerRosBridge::gsImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);
        pipeline_.write(image);
    } catch (cv_bridge::Exception &e) {
        // print error message
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

