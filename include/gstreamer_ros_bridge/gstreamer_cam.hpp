#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
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

namespace gstreamer_ros_bridge
{
    class GStreamerCam : public nodelet::Nodelet
    {
    public:
        GStreamerCam() = default;
        ~GStreamerCam();
        void Update(const ros::TimerEvent&);

        // gstreamer
        GstElement *pipeline_;
        GstElement *appsink_;

    private:
        virtual void onInit() override;
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
        ros::Publisher rosImagePub_;
        ros::NodeHandle nh_;

        // image processing
        cv::Mat frame_;
        const gchar *format_{nullptr};

        // camera params
        std::string camera_location_, camera_format_, camera_topic_;
        int camera_width_, camera_height_, camera_fps_;

        ros::Timer update_timer_;
    };
} // namespace gstreamer_ros_bridge

#endif 
