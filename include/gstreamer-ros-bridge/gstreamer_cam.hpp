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
    COMPOSITION_PUBLIC
    GStreamerCam();
    ~GStreamerCam();

    GstElement *pipeline_;
    GstElement *appsink_;

    private:
        void Update(const rclcpp::TimerEvent&);

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
        image_transport::ImageTransport it_;
        image_transport::Publisher ros_image_pub_;
        rclcpp::TimerBase update_timer_;

        // null gchar initialization
        const gchar *format_{nullptr};

        std::string camera_location_, camera_format_, camera_topic_;
        int camera_width_, camera_height_, camera_fps_;
        cv::Mat frame_; // published frame
};

}

#endif // GSTREAMER_ROS_BRIDGE_GSTREAMER_CAM_HPP