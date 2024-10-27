#pragma once

#include <string>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

class GstreamerPublisher {
public:
    GstreamerPublisher(ros::NodeHandle nh, const std::string &out_topic);
    ~GstreamerPublisher();

    GstreamerPublisher(const GstreamerPublisher&) = delete;
    GstreamerPublisher& operator=(const GstreamerPublisher&) = delete;

    /**
     * @brief Initialize pipelines.
     * 
     * @return True if initialization was successful.
     */
    bool init();
    /**
     * @brief Try to receive and publish a single frame to the ROS topic.
     *
     * @return Whether the frame was processed properly.
     */
    bool tryProcessFrame();

private:
    ros::NodeHandle nh_;

    cv::VideoCapture gstIn_;
    ros::Publisher imgOutPub_;
};

