#include "gstreamer_bridge.hpp"

void GstreamerRosBridge::setCameraParams()
{   
    cameraInfo_.header.frame_id = "camera";
    cameraInfo_.height = 480;
    cameraInfo_.width = 640;
    cameraInfo_.distortion_model = "plumb_bob";
    cameraInfo_.D = {0.0, 0.0, 0.0, 0.0, 0.0};
    cameraInfo_.K = {640.0, 0.0, 320.0, 0.0, 480.0, 240.0, 0.0, 0.0, 1.0};
    cameraInfo_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    cameraInfo_.P = {640.0, 0.0, 320.0, 0.0, 0.0, 480.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
}

void GstreamerRosBridge::pubCameraImage()
{
    // try grabbing a frame
    if (!cap_.grab()) {
        ROS_ERROR("Failed to grab a frame from the camera");
        return; 
    }

    // retrieve the frame and update frame_
    if (!cap_.retrieve(frame_)) {
        ROS_ERROR("Failed to retrieve frame from the camera");
        return;  
    }
    if (frame_.empty()) {
        ROS_ERROR("No image captured (it's gg)");
        return;
    }
	
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
    ros::Time current_time = ros::Time::now();
    msg->header.stamp = current_time;
    cameraInfo_.header.stamp = current_time;

    rosImagePub_.publish(msg);
    rosCameraInfoPub_.publish(cameraInfo_);
}

