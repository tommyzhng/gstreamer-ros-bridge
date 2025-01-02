#ifndef GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP
#define GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <gst/gst.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <cstdlib>

/* 
    this node takes a image rostopic and pushes it through a gstreamer pipeline
*/
namespace gstreamer_ros_bridge
{
    class GStreamerRosBridge : public nodelet::Nodelet
    {
    public:
        GStreamerRosBridge() = default;
        ~GStreamerRosBridge();
        // ros_rate for fps workaround
        ros::Rate ros_rate_{30};
    private:
        virtual void onInit() override;
        /**
        * @brief function to recieve from ros image topic and publish to pipeline to peer 
        * @param msg - image message from ros
        * @return void
        */    
        void GsImageCallback(const sensor_msgs::ImageConstPtr &msg);
        
        ros::Subscriber gsImageSub_;

        // gstreamer pipeline
        cv::VideoWriter pipeline_;
        cv::Mat frame_;

        // gstreamer param member vars
        int gst_width_{640}, gst_height_{480};

    };
} // namespace gstreamer_ros_bridge

#endif //GSTREAMER_ROS_BRIDGE_GSTREAMER_BRIDGE_HPP