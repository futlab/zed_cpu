#ifndef ZED_H
#define ZED_H
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <memory>
#include <utility>

class ZEDCamera
{
public:
    ZEDCamera(int deviceId, int resolution = 3, double frame_rate = 10.0);
    void setResolution(int type);
    void setFrameRate(double frame_rate);
    inline double getFrameRate() { return frame_rate_; }
    bool getImages(cv::Mat& left_image, cv::Mat& right_image);
    inline bool getImage(cv::Mat& raw) { return camera_.read(raw); }
    inline bool ok() { return camera_.isOpened(); }
    static const std::vector<std::pair<int, int>> resolutions;
private:
    cv::VideoCapture camera_;
    int width_;
    int height_;
    double frame_rate_;
    bool cv_three_;
};

#endif // ZED_H
