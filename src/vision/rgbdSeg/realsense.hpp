#ifndef REALSENSE_HPP
#define REALSENSE_HPP

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <functional>

class RealSense {
private:
    rs2::pipeline pipeline;
    rs2::config config;
    rs2::pipeline_profile profile;

    void configureCameraSettings();
    void warmUpPipeline();

public:
    RealSense();
    void startPipeline();
    rs2::pipeline_profile getPipelineProfile() const;
    rs2::pipeline& getPipeline();
    void captureFrames(const std::function<void(const rs2::frameset&)>& frameHandler);
};

std::pair<cv::Mat, cv::Mat> depthMatFrameProcess(const rs2::frameset& frames);
cv::Mat minMaxScale(const cv::Mat& inputImage, int cvType);
cv::Mat normalizeRGB(cv::Mat& rgbImage);
cv::Mat normalizeDepth(cv::Mat& depthImage);
rs2::frame preprocessDepth(rs2::frame& depth_frame);
std::pair<cv::Mat, cv::Mat> postProcessFrames(cv::Mat& color_mat, cv::Mat& depth_mat);

template<typename T>
cv::Mat applyNonZeroMedianFilter(const cv::Mat& image, int factor);
#endif
