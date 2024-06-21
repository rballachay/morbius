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
    std::vector<int64_t> inputShape;
    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;

public:
    OnnxPipeline(const char* modelPath);
    cv::Mat forward(cv::Mat inputs);
    std::vector<float> runInference(const cv::Mat& inputImage);
    cv::Mat getMaxChannelIndices(const cv::Mat& multiChannelMat);
    cv::Mat displayMaxChannelIndices(const cv::Mat& multiChannelMat);
};

#endif
