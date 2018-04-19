#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include "zed.h"
#include "zed_ros.h"


void ZedCameraROS::loadParams()
{
    ros::NodeHandle privateNh("~");
    privateNh.param("resolution", resolution_, 1);
    if (resolution_ < 0 || resolution_ >= ZEDCamera::resolutions.size())
        ROS_FATAL("Wrong resolution specified: %d", resolution_);
    auto &rp = ZEDCamera::resolutions[resolution_];
    width_ = rp.first / 2;
    height_ = rp.second;
    privateNh.param("frame_rate", frameRate_, 15.0);
    privateNh.param("config_file_location", config_file_location_, std::string("~/SN1267.conf"));
    privateNh.param("left_frame_id", left_frame_id_, std::string("left_camera"));
    privateNh.param("right_frame_id", right_frame_id_, std::string("right_camera"));
    privateNh.param("show_image", showImage_, false);
    privateNh.param("load_zed_config", load_zed_config_, true);
    privateNh.param("device_id", deviceId_, 0);
    privateNh.param("pub_throttle", pubThrottleHz_, 0.0);
}

void ZedCameraROS::loadConfig()
{
    ros::NodeHandle nh;
    ROS_INFO("Try load camera calibration files");
    if (load_zed_config_) {
        ROS_INFO("Loading from zed calibration files");
        // get camera info from zed
        try {
            getZedCameraInfo(config_file_location_, resolution_, leftInfo_, rightInfo_);
        }
        catch (std::runtime_error& e) {
            ROS_INFO("Can't load camera info");
            ROS_ERROR("%s", e.what());
            throw e;
        }
    } else {
        ROS_INFO("Loading from ROS calibration files");
        // get config from the left, right.yaml in config
        camera_info_manager::CameraInfoManager info_manager(nh);
        info_manager.setCameraName("zed/left");
        info_manager.loadCameraInfo("package://zed_cpu/config/left" + std::to_string((int)height_) + ".yaml");
        leftInfo_ = info_manager.getCameraInfo();

        info_manager.setCameraName("zed/right");
        info_manager.loadCameraInfo("package://zed_cpu/config/right" + std::to_string((int)height_) + ".yaml");
        rightInfo_ = info_manager.getCameraInfo();

        leftInfo_.header.frame_id = left_frame_id_;
        rightInfo_.header.frame_id = right_frame_id_;
    }
    ROS_INFO("Got camera calibration files");
}

void ZedCameraROS::advertise(bool advertiseThrottle)
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    leftImagePub_ = it.advertise("left/image_raw", 1);
    rightImagePub_ = it.advertise("right/image_raw", 1);
    leftCamInfoPub_ = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    rightCamInfoPub_ = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);

    if (advertiseThrottle)
        throttlePub_ = it.advertise("image_throttle", 1);

}

void ZedCameraROS::publish(const cv::Mat &leftImage, const cv::Mat &rightImage)
{
    ros::Time now = ros::Time::now();
    if (showImage_) {
        cv::imshow("left", leftImage);
        cv::imshow("right", rightImage);
    }
    if (leftImagePub_.getNumSubscribers() > 0) {
        publishImage(leftImage, leftImagePub_, "left_frame", now);
    }
    if (rightImagePub_.getNumSubscribers() > 0) {
        publishImage(rightImage, rightImagePub_, "right_frame", now);
    }
    if (leftCamInfoPub_.getNumSubscribers() > 0) {
        publishCamInfo(leftCamInfoPub_, leftInfo_, now);
    }
    if (rightCamInfoPub_.getNumSubscribers() > 0) {
        publishCamInfo(rightCamInfoPub_, rightInfo_, now);
    }
}

void ZedCameraROS::zedRosLoop()
{
    ROS_INFO("Try to initialize the camera");
    ZEDCamera zed(deviceId_, resolution_, frameRate_);
    ROS_INFO("Initialized the camera");

    advertise(pubThrottleHz_ > 0);
    loadConfig();
    // loop to publish images;
    cv::Mat image;
    ros::Rate r(frameRate_);
    ros::Duration throttlePeriod(pubThrottleHz_ > 0 ? 1.0 / pubThrottleHz_ : 0.0);
    ros::Time nextThrottle = ros::Time::now();
    cv::Rect left_rect(0, 0, width_, height_), right_rect(width_, 0, width_, height_);
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        if (!zed.getImage(image)) {
            ROS_INFO_ONCE("Can't find camera");
        } else {
            ROS_INFO_ONCE("Success, found camera");
        }
        cv::Mat leftImage = image(left_rect), rightImage = image(right_rect);
        if(!throttlePeriod.isZero() && throttlePub_.getNumSubscribers() > 0 && now > nextThrottle) {
            publishImage(image, throttlePub_, "frame", now);
            nextThrottle = now + throttlePeriod;
        }

        publish(leftImage, rightImage);
        r.sleep();
        // since the frame rate was set inside the camera, no need to do a ros sleep
    }
}

void ZedCameraROS::rosRosLoop()
{
    advertise(false);
    loadConfig();
    ros::NodeHandle nh;
    ros::Subscriber leftImageSub_, rightImageSub_;
    leftImageSub_ = nh.subscribe("left/image_throttle", 10, &ZedCameraROS::leftImageCallback, this),
    rightImageSub_ = nh.subscribe("right/image_throttle", 10, &ZedCameraROS::rightImageCallback, this);

    ros::spin();
}

void ZedCameraROS::leftImageCallback(const sensor_msgs::Image &msg)
{
    ROS_INFO_ONCE("Repeater: Left image received");
    left_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    process();
}

void ZedCameraROS::rightImageCallback(const sensor_msgs::Image &msg)
{
    ROS_INFO_ONCE("Repeater: Right image received");
    right_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    process();
}

void ZedCameraROS::process()
{
    if (left_.empty() || right_.empty())
        return;
    publish(left_, right_);
    left_ = cv::Mat();
    right_ = cv::Mat();
}

ZedCameraROS::ZedCameraROS() {
    loadParams();
}

void ZedCameraROS::getZedCameraInfo(std::string config_file, int resolution, sensor_msgs::CameraInfo &left_info, sensor_msgs::CameraInfo &right_info) {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);
    std::string left_str = "LEFT_CAM_";
    std::string right_str = "RIGHT_CAM_";
    std::string reso_str = "";

    switch (resolution) {
    case 0: reso_str = "2K"; break;
    case 1: reso_str = "FHD"; break;
    case 2: reso_str = "HD"; break;
    case 3: reso_str = "VGA"; break;
    }
    // left value
    double l_cx = pt.get<double>(left_str + reso_str + ".cx");
    double l_cy = pt.get<double>(left_str + reso_str + ".cy");
    double l_fx = pt.get<double>(left_str + reso_str + ".fx");
    double l_fy = pt.get<double>(left_str + reso_str + ".fy");
    double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
    double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
    // right value
    double r_cx = pt.get<double>(right_str + reso_str + ".cx");
    double r_cy = pt.get<double>(right_str + reso_str + ".cy");
    double r_fx = pt.get<double>(right_str + reso_str + ".fx");
    double r_fy = pt.get<double>(right_str + reso_str + ".fy");
    double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
    double r_k2 = pt.get<double>(right_str + reso_str + ".k2");

    // get baseline and convert mm to m
    boost::optional<double> baselineCheck;
    double baseline = 0.0;
    // some config files have "Baseline" instead of "BaseLine", check accordingly...
    if (baselineCheck = pt.get_optional<double>("STEREO.BaseLine")) {
        baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
    }
    else if (baselineCheck = pt.get_optional<double>("STEREO.Baseline")) {
        baseline = pt.get<double>("STEREO.Baseline") * 0.001;
    }
    else
        throw std::runtime_error("baseline parameter not found");

    // get Rx and Rz
    double rx = pt.get<double>("STEREO.RX_"+reso_str);
    double rz = pt.get<double>("STEREO.RZ_"+reso_str);
    double ry = pt.get<double>("STEREO.CV_"+reso_str);

    // assume zeros, maybe not right
    double p1 = 0, p2 = 0, k3 = 0;

    left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // distortion parameters
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    left_info.D.resize(5);
    left_info.D[0] = l_k1;
    left_info.D[1] = l_k2;
    left_info.D[2] = k3;
    left_info.D[3] = p1;
    left_info.D[4] = p2;

    right_info.D.resize(5);
    right_info.D[0] = r_k1;
    right_info.D[1] = r_k2;
    right_info.D[2] = k3;
    right_info.D[3] = p1;
    right_info.D[4] = p2;

    // Intrinsic camera matrix
    // 	[fx  0 cx]
    // K =  [ 0 fy cy]
    //	[ 0  0  1]
    left_info.K.fill(0.0);
    left_info.K[0] = l_fx;
    left_info.K[2] = l_cx;
    left_info.K[4] = l_fy;
    left_info.K[5] = l_cy;
    left_info.K[8] = 1.0;

    right_info.K.fill(0.0);
    right_info.K[0] = r_fx;
    right_info.K[2] = r_cx;
    right_info.K[4] = r_fy;
    right_info.K[5] = r_cy;
    right_info.K[8] = 1.0;

    // rectification matrix
    // Rl = R_rect, R_r = R * R_rect
    // since R is identity, Rl = Rr;
    left_info.R.fill(0.0);
    right_info.R.fill(0.0);

    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 5 * rx, ry, rz);
    cv::Mat rmat(3, 3, CV_64F);
    cv::Rodrigues(rvec, rmat);
    int id = 0;
    cv::MatIterator_<double> it, end;
    //std::copy(rmat.begin<double>(), rmat.end<double>(), right_info.R.begin());
    for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++){
        //left_info.R[id] = *it;
        right_info.R[id] = *it;
    }
    left_info.R[0] = 1.0;
    left_info.R[4] = 1.0;
    left_info.R[8] = 1.0;
    /*right_info.R[0] = 1.0;
        right_info.R[4] = 1.0;
        right_info.R[8] = 1.0;*/


    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    left_info.P.fill(0.0);
    left_info.P[0] = l_fx;
    left_info.P[2] = l_cx;
    left_info.P[5] = l_fy;
    left_info.P[6] = l_cy;
    left_info.P[10] = 1.0;

    right_info.P.fill(0.0);
    right_info.P[0] = r_fx;
    right_info.P[2] = r_cx;
    right_info.P[3] = (-1 * l_fx * baseline);
    right_info.P[5] = r_fy;
    right_info.P[6] = r_cy;
    right_info.P[10] = 1.0;

    left_info.width = right_info.width = width_;
    left_info.height = right_info.height = height_;

    left_info.header.frame_id = left_frame_id_;
    right_info.header.frame_id = right_frame_id_;
}

void ZedCameraROS::publishCamInfo(const ros::Publisher &pub_cam_info, sensor_msgs::CameraInfo &cam_info_msg, ros::Time now) {
    cam_info_msg.header.stamp = now;
    pub_cam_info.publish(cam_info_msg);
}

void ZedCameraROS::publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t) {
    cv_bridge::CvImage cv_image;
    cv_image.image = img;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.header.frame_id = img_frame_id;
    cv_image.header.stamp = t;
    img_pub.publish(cv_image.toImageMsg());
}
