#include "gstreamer_bridge.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gstreamer_bridge_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(30);

    GstreamerRosBridge bridge(nh);

    while (ros::ok())
    {
        bridge.Update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

