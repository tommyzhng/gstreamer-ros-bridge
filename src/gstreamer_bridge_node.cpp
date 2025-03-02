#include <gstreamer_bridge.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gstreamer_bridge_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    nodelet.load(ros::this_node::getName(),
                "gstreamer_ros_bridge/GStreamerRosBridge",
                remap, nargv);

    ros::spin();
    return 0;
}

