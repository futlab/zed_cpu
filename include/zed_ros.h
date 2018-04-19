#ifndef ZED_ROS_H
#define ZED_ROS_H

#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

/**
 * @brief       the camera ros warpper class
 */
class ZedCameraROS {
public:

    /**
     * @brief      { function_description }
     *
     * @param[in]  resolution  The resolution
     * @param[in]  frame_rate  The frame rate
     */
    ZedCameraROS();

    /**
     * @brief      Gets the camera information From Zed config.
     *
     * @param[in]  config_file         The configuration file
     * @param[in]  resolution          The resolution
     * @param[in]  left_cam_info_msg   The left camera information message
     * @param[in]  right_cam_info_msg  The right camera information message
     */
    void getZedCameraInfo(std::string config_file, int resolution, sensor_msgs::CameraInfo& left_info, sensor_msgs::CameraInfo& right_info);

    /**
     * @brief      { publish camera info }
     *
     * @param[in]  pub_cam_info  The pub camera information
     * @param[in]  cam_info_msg  The camera information message
     * @param[in]  now           The now
     */
    void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now);

    /**
     * @brief      { publish image }
     *
     * @param[in]  img           The image
     * @param      img_pub       The image pub
     * @param[in]  img_frame_id  The image frame identifier
     * @param[in]  t             { parameter_description }
     */
    void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t);
    void zedRosLoop();
    void rosRosLoop();
private:
    int deviceId_, resolution_;
    double frameRate_;
    bool showImage_, load_zed_config_;
    double width_, height_;
    double pubThrottleHz_;
    std::string left_frame_id_, right_frame_id_;
    std::string config_file_location_;
    sensor_msgs::CameraInfo leftInfo_, rightInfo_;
    ros::Publisher leftCamInfoPub_, rightCamInfoPub_;
    cv::Mat left_, right_;
    image_transport::Publisher leftImagePub_, rightImagePub_, throttlePub_;

    void loadParams();
    void loadConfig();
    void advertise(bool advertiseThrottle);
    void publish(const cv::Mat &leftImage, const cv::Mat &rightImage);
    void leftImageCallback(const sensor_msgs::Image &msg);
    void rightImageCallback(const sensor_msgs::Image &msg);
    void process();
};

#endif // ZED_ROS_H
