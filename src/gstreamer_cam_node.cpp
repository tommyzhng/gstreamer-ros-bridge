#include "gstreamer_cam.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gstreamer_cam_node");
    ros::NodeHandle nh("~");

    gst_init(&argc, &argv);
    GStreamerCam cam(nh);
    ros::Rate rate(cam.GetFrameRate());

    while (ros::ok())
    {
        cam.Update();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}