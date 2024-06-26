#ifndef ONNPIPELINE_HPP
#define ONNPIPELINE_HPP

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class OnnxPipeline {
private:
    const char* modelPath;
    Ort::Env env;
    Ort::SessionOptions sessionOptions;
    Ort::Session session;
    std::vector<int64_t> inputShapeRGB;
    std::vector<int64_t> inputShapeD;
    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;
    std::vector<cv::Vec3b> colorSpace; 
    std::vector<std::string> classNames;

public:
    OnnxPipeline(const char* modelPath);
    cv::Mat forward(cv::Mat inputsRGB, cv::Mat inputsD);
    cv::Mat getMaxChannelIndices(const cv::Mat& multiChannelMat);
    cv::Mat displayMaxChannelIndices(const cv::Mat& multiChannelMat);
    cv::Mat blendImages(const cv::Mat& rawImage, const cv::Mat& maskImage);
};

#endif
