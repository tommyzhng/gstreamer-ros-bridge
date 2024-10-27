#include "gstreamer_publisher.cpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gstreamer_publisher_node");
    ros::NodeHandle nh("~");

    GstreamerPublisher gstPub(nh, "/republished");
    if (!gstPub.init()) {
        ROS_ERROR("Failed to initialize gstreamer publisher, exiting");
        return 1;
    }

    while (ros::ok()) {
        if (!gstPub.tryProcessFrame()) {
            ROS_ERROR("Stopping gstreamer publisher on error...");
            return 1;
        }
        ros::spinOnce();
    }
    return 0;
}
