#include "zed.h"

using namespace std;

const vector<pair<int, int>> ZEDCamera::resolutions{{4416, 1242}, {3840, 1080}, {2560, 720}, {1344, 376}};

ZEDCamera::ZEDCamera(int deviceId, int resolution, double frame_rate): camera_(deviceId), frame_rate_(frame_rate) {
    setResolution(resolution);
    setFrameRate(frame_rate);

    //std::cout << "Stereo Camera Set Resolution: " << camera_->get(WIDTH_ID) << "x" << camera_->get(HEIGHT_ID) << std::endl;
    // std::cout << "Stereo Camera Set Frame Rate: " << camera_->get(FPS_ID) << std::endl;
}

void ZEDCamera::setResolution(int type) {
    assert(type >= 0 && type < 4);
    const auto &rp = resolutions[type];
    width_ = rp.first;
    height_ = rp.second;

    camera_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);

    // make sure that the number set are right from the hardware
    width_ = camera_.get(cv::CAP_PROP_FRAME_WIDTH);
    height_ = camera_.get(cv::CAP_PROP_FRAME_HEIGHT);
}

void ZEDCamera::setFrameRate(double frame_rate) {
    camera_.set(cv::CAP_PROP_FPS, frame_rate);
    frame_rate_ = camera_.get(cv::CAP_PROP_FPS);
}

bool ZEDCamera::getImages(cv::Mat &left_image, cv::Mat &right_image) {
    cv::Mat raw;
    if (camera_.grab()) {
        camera_.retrieve(raw);
        cv::Rect left_rect(0, 0, width_ / 2, height_);
        cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
        left_image = raw(left_rect);
        right_image = raw(right_rect);
        return true;
    } else {
        return false;
    }
}


