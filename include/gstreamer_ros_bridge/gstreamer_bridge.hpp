#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <cstdlib>


/* 
    this node takes a image rostopic and pushes it through a gstreamer pipeline
    it also starts a video feed from the camera and pushes it onto a camera node for other topics to digest
*/

class GstreamerRosBridge
{
public:
    GstreamerRosBridge(ros::NodeHandle &nh);
    ~GstreamerRosBridge();
    void Update();
private:
    /**
    * @brief set cam params for camera/camera_info 
    */
    void setCameraParams();

    /** 
    * @brief images from camera onto camera/image_rect
    */
    void pubCameraImage();

    /**
    * @brief function to recieve from ros image topic and publish to pipeline to peer 
    * @param msg - image message from ros
    * @return void
    */    
    void gsImageCallback(const sensor_msgs::ImageConstPtr &msg);

    ros::Subscriber gsImageSub_;
    ros::Publisher rosCameraInfoPub_;
    ros::Publisher rosImagePub_;
    sensor_msgs::CameraInfo cameraInfo_;
    ros::NodeHandle nh_;
    
    // gstreamer pipeline
    cv::VideoCapture cap_;
    cv::Mat frame_;
    cv::VideoWriter pipeline_;

    // gstreamer param member vars
    int gst_width_{640}, gst_height_{480};
};

#endif //GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP