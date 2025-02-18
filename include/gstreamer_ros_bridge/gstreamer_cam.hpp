#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <cstdlib>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <image_transport/image_transport.h>

/* 
    this node starts a gstreamer video feed from the camera and pushes it onto a camera topic
*/

class GStreamerCam
{
public:
    GStreamerCam(ros::NodeHandle &nh);
    ~GStreamerCam();
    void Update();
    int GetFrameRate() const;

    GstElement *pipeline_;
    GstElement *appsink_;

private:
    /**
    * @brief set cam params for camera/camera_info 
    */
    void SetCameraParams();
    /**
    * @brief opens the camera pipeline
    */
    void InitializeGStreamer();
    /** 
    * @brief images from camera onto camera/image_rect
    */
    void PubCameraImage();

    void ConvertImage(GstMapInfo &map_info);

    sensor_msgs::CameraInfo cameraInfo_;
    ros::Publisher rosCameraInfoPub_;
    image_transport::Publisher rosImagePub_;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    cv::Mat frame_; // published frame
    // null gchar initialization
    const gchar *format_{nullptr};

    std::string camera_location_, camera_format_, camera_topic_;
    int camera_width_, camera_height_, camera_fps_;
};

#endif 
