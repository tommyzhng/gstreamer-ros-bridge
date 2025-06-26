#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/Image.h>

class GStreamerPublisher : public rclcpp::Node
{
public:
    GStreamerPublisher(std::shared_ptr<rclcpp::Node> node, const std::string &out_topic);
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
     bool tryProcessFrame();

private:
    std::shared_ptr<rclcpp::Node> node;
    
    cv::VideoCapture gstIn_;
    rclcpp::Publisher<sensor_msgs::msg::Image> imgOutPub_;

};