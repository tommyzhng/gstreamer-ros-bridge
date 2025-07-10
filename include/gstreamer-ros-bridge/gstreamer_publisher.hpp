#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_PUBLISHER_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_PUBLISHER_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

class GStreamerPublisher : public rclcpp::Node
{
public:
    GStreamerPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), const std::string &out_topic);
    ~GStreamerPublisher();

    GStreamerPublisher(const GStreamerPublisher&) = delete;
    GStreamerPublisher& operator=(const GStreamerPublisher&) = delete;

    /**
     * @brief Initialize the pipeline.
     * 
     * @param port UDP port that the stream will be read from.
     * @return True if initialization was successful.
     */
     bool init(int port);

    /**
     * @brief Try to receive and publish a single frame to the ROS topic.
     * 
     * @return Whether the frame was processed properly.
     */
     bool try_process_frame();

private:
    
    cv::VideoCapture gst_in_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_out_pub_;

};

#endif
