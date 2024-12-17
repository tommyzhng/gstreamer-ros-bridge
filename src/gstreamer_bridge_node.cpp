#include "gstreamer_bridge.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gstreamer_bridge_node");
    ros::NodeHandle nh("~");

    GStreamerRosBridge bridge(nh);

    while (ros::ok())
    {
        ros::spinOnce();
        bridge.ros_rate_.sleep();
    }
    return 0;
}

