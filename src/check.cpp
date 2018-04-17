#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "zed.h"

void testFps(int fps, image_transport::Publisher &pub, ZEDCamera &cam, bool noStop)
{
    cam.setFrameRate(fps);
    ROS_INFO("Set fps to %d (actual %d)", fps, (int)cam.getFrameRate());
    ros::Rate rate(fps);
    cv::Mat raw;
    int dropped = 0, recv = 0;
    for (ros::Time end = ros::Time::now() + ros::Duration(1.0); ros::Time::now() < end || (noStop && ros::ok()); ) {
        if(!cam.getImage(raw)) {
            ++dropped;
            continue;
        }
        if(pub.getNumSubscribers() > 0)
            pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw).toImageMsg());
        ++recv;
        rate.sleep();
    }
    ROS_INFO("Rate received %d, dropped %d", recv, dropped);
}

void testResolution(int res, int fps, image_transport::Publisher &pub, ZEDCamera &cam, bool noStop)
{
    auto &rp = cam.resolutions[res];
    ROS_INFO("Set resolution to %d x %d", rp.first, rp.second);
    cam.setResolution(res);
    if (fps >= 0)
        testFps(fps, pub, cam, noStop);
    else for (fps = 5; fps < 30 && ros::ok(); fps += 5)
        testFps(fps, pub, cam, noStop);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "check");
    ros::NodeHandle nh, nhp("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 1);
    int res, fps;
    nhp.param("res", res, -1);
    nhp.param("fps", fps, -1);
    bool noStop = res >= 0 && fps >= 0;

    for (int id = 0; id < 10; id++) {
        ZEDCamera cam(id);
        if (cam.ok()) {
            ROS_INFO("Found camera with id %d", id);
            if (res >= 0)
                testResolution(res, fps, pub, cam, noStop);
            else for (res = 3; res >= 0; --res)
                testResolution(res, fps, pub, cam, noStop);
            return EXIT_SUCCESS;
        }
    }
    ROS_WARN("No camera found");
    return EXIT_SUCCESS;
}
