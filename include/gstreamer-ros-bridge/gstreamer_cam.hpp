#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.h>
#include <cstdlib>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <image_transport/image_transport.hpp>

namespace gstreamer_ros_bridge
{

    /*
        This node starts a gstreamer video feed from the camera and pushes it onto a camera topic
    */

class GStreamerCam : public rclcpp::Node
{
public:
    GStreamerCam();
    ~GStreamerCam();

    GstElement *pipeline_;
    GstElement *appsink_;

    private:
        void Update(const rclcpp::TimerEvent&);

        /**
         * @brief set cam params for sensor_msgs/msg/camera_info
         */
        void SetCameraParams();
        /**
         * @brief opens the camera pipeline
         */
        void InitializeGStreamer();
        /**
         * @brief images from camera onto sensor_msgs/msg/Image
         */
        void PubCameraImage();

        void ConvertImage(GstMapInfo &map_info);

        sensor_msgs::msg::CameraInfo cameraInfo_;
        cv::Mat frame_; // published frame
        rclcpp::Publisher<cameraInfo_> rosCameraInfoPub_;
        image_transport::Publisher<cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_).toImageMsg()> rosImagePub_;

        rclcpp::TimerBase update_timer_;
        image_transport::ImageTransport it_;
        
        // null gchar initialization
        const gchar *format_{nullptr};

        std::string camera_location_, camera_format_, camera_topic_;
        int camera_width_, camera_height_, camera_fps_;
};

}

#endif