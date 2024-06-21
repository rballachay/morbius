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

cv::Mat depthMatFrameProcess(const rs2::frameset& frames);
cv::Mat maxMinScaleChannels(const cv::Mat& inputImage);

#endif
