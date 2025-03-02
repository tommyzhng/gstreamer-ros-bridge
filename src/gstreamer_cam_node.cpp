#include <gstreamer_cam.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gstreamer_cam_node");
    gst_init(&argc, &argv);
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;

    nodelet.load(ros::this_node::getName(),
                "gstreamer_ros_bridge/GStreamerCam",
                remap, nargv);

    ros::spin();
    return 0;
}