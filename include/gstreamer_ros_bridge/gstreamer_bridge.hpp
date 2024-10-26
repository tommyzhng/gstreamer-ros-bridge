#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


/* 
this node takes a image rostopic and pushes it through a gstreamer pipeline
*/

class GstreamerRosBridge
{
public:
    GstreamerRosBridge(ros::NodeHandle &nh);
    ~GstreamerRosBridge();
private:
    void startImageCapture();
    void gsImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void pubRosImage(const cv::Mat &image);

    ros::Subscriber gsImageSub_;
    ros::Publisher rosCameraInfoPub_;
    ros::Publisher rosImagePub_;
    sensor_msgs::CameraInfo cameraInfo_;
    ros::NodeHandle nh_;
    
    // gstreamer pipeline
    cv::VideoCapture cap_;
    cv::VideoWriter pipeline_;
};

#endif //GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP