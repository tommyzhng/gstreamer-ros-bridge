#include "gstreamer_publisher.cpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gstreamer_publisher_node");
    ros::NodeHandle nh("~");

    std::string topicName = "/gstreamer_received";
    nh.getParam("topic_name", topicName);
    int port = 5602;
    nh.getParam("port", port);
    std::string customPipeline;
    nh.getParam("pipeline", customPipeline);

    GstreamerPublisher gstPub(nh, topicName);
    if (!customPipeline.empty()) {
        if (!gstPub.init(customPipeline)) {
            ROS_ERROR("Failed to initialize gstreamer publisher with custom pipeline, exiting");
            return 1;
        }
    }
    else {
        if (!gstPub.init(port)) {
            ROS_ERROR("Failed to initialize gstreamer publisher, exiting");
            return 1;
        }
    }

    // Max possible rate
    // Usually won't matter since reading the frame blocks anyway
    ros::Rate rate(60);
    while (ros::ok()) {
        if (!gstPub.tryProcessFrame()) {
            ROS_ERROR("Stopping gstreamer publisher on error...");
            return 1;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
