#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "zed_ros.h"

int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, "zed_camera");
        ZedCameraROS zed_ros;
        zed_ros.rosRosLoop();
        return EXIT_SUCCESS;
    }
    catch(std::runtime_error& e) {
        ros::shutdown();
        return EXIT_FAILURE;
    }
}
