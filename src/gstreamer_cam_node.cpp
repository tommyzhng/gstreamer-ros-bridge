#include "gstreamer_cam.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gstreamer_cam_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(60);

    gst_init(&argc, &argv);
    GStreamerCam cam(nh);

    while (ros::ok())
    {
        cam.Update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}