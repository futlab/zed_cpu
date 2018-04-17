#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "zed.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "check");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 1);

    for (int id = 0; id < 10; id++) {
        ZEDCamera cam(id);
        if (cam.ok()) {
            ROS_INFO("Found camera with id %d", id);
            for (int res = 3; res >= 0; --res) {
                auto &rp = cam.resolutions[res];
                ROS_INFO("Set resolution to %d x %d", rp.first, rp.second);
                cam.setResolution(res);
                for (int fps = 5; fps < 30 && ros::ok(); fps += 5) {
                    ROS_INFO("Set fps to %d", fps);
                    cam.setFrameRate(fps);
                    ros::Rate rate(fps);
                    cv::Mat raw;
                    int dropped = 0, recv = 0;
                    for (ros::Time end = ros::Time::now() + ros::Duration(1.0); ros::Time::now() < end; ) {
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
            }
            return EXIT_SUCCESS;
        }
    }
    ROS_WARN("No camera found");
    return EXIT_SUCCESS;
}
