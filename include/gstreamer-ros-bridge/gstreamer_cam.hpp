#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cstdlib>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace gstreamer_ros_bridge
{

    /*
        This node starts a gstreamer video feed from the camera and pushes it onto a camera topic
    */

class GStreamerCam : public rclcpp::Node
{
public:
    explicit GStreamerCam(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~GStreamerCam();

    GstElement *pipeline_;
    GstElement *appsink_;

    private:
        void update();

        /**
         * @brief set cam params for sensor_msgs/msg/camera_info
         */
        void set_cam_info();
        /**
         * @brief opens the camera pipeline
         */
        void initialize_gstreamer();
        /**
         * @brief images from camera onto sensor_msgs/msg/Image
         */
        void pub_camera_image();

        void convert_image(GstMapInfo &map_info);

        sensor_msgs::msg::CameraInfo camera_info_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ros_camera_info_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_pub_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        // null gchar initialization
        const gchar *format_{nullptr};

        std::string camera_location_, camera_format_, camera_topic_;
        int camera_width_, camera_height_, camera_fps_;
        cv::Mat frame_; // published frame
};

}

#endif // GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP