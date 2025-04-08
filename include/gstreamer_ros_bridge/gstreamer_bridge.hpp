#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <gst/gst.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/SetBool.h>
#include <nodelet/nodelet.h>
#include <nodelet/loader.h>
#include <cstdlib>
#include <functional>

namespace gstreamer_ros_bridge
{

/* 
    this node takes a image rostopic and pushes it through a gstreamer pipeline
*/

class GStreamerRosBridge : public nodelet::Nodelet
{
public:
    GStreamerRosBridge() = default;
    ~GStreamerRosBridge();

    ros::NodeHandle nh_;

private:
    virtual void onInit() override;

    cv::Mat resizeAndPad(cv::Mat &image, int target_width, int target_height);

    bool SetStreamOnCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

    /**
    * @brief function to recieve from ros image topic and publish to pipeline to peer 
    * @param msg - image message from ros
    * @return void
    */    
    void GsImageCallback(const sensor_msgs::ImageConstPtr &msg);
    
    ros::Subscriber gsImageSub_;
    ros::ServiceServer set_stream_on_srv_;

    // gstreamer pipeline
    cv::VideoWriter pipeline_;
    cv::Mat frame_;

    // gstreamer param member vars
    int gst_width_{640}, gst_height_{480};

    bool stream_on_ = true;
};

}

#endif //GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP