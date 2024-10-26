#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <string>

/* 
this node takes a image rostopic and pushes it through a gstreamer pipeline
*/

class GstreamerRosBridge
{
public:
    GstreamerRosBridge(ros::NodeHandle &nh);
    ~GstreamerRosBridge();

    void sendImage(const sensor_msgs::CompressedImageConstPtr &msg);

private:
    void startImageCapture();
    void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg);

    ros::NodeHandle nh_;
    GstElement *pipeline;


}
#endif; //GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP