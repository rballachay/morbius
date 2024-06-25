#include "onnxpipeline.hpp"
#include <vector>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <random>

OnnxPipeline::OnnxPipeline(const char* modelPath) 
    : modelPath(modelPath), env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference"), session(nullptr),
    sessionOptions(nullptr), inputShapeRGB({1, 3, 480, 640}), inputShapeD({1, 1, 480, 640}), 
    inputNames({ "rgb", "depth" }), outputNames({ "output" }), colorSpace(40), classNames(40){
    
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Create an ONNX Runtime session
    session = Ort::Session(env, modelPath, sessionOptions);

    // Get input node names and shapes
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_input_nodes = session.GetInputCount();

    // Define colors for each channel index
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);

    // Prepare colors vector with random colors for each index
    for (int i = 0; i < 40; ++i) {
        colorSpace[i] = {static_cast<uchar>(dis(gen)), static_cast<uchar>(dis(gen)), static_cast<uchar>(dis(gen))};
    }
    // classes can be loaded from this link into python
    // https://github.com/TUI-NICR/nicr-scene-analysis-datasets/blob/main/nicr_scene_analysis_datasets/datasets/nyuv2/classMapping40.mat
    classNames = {
        "wall", "floor", "cabinet", "bed", "chair", "sofa", "table", "door", "window",
        "bookshelf", "picture", "counter", "blinds", "desk", "shelves", "curtain",
        "dresser", "pillow", "mirror", "floor mat", "clothes", "ceiling", "books",
        "refrigerator", "television", "paper", "towel", "shower curtain", "box",
        "whiteboard", "person", "night stand", "toilet", "sink", "lamp", "bathtub",
        "bag", "otherstructure", "otherfurniture", "otherprop"
    };

}

cv::Mat OnnxPipeline::forward(cv::Mat inputsRGB, cv::Mat inputsD){

    cv::Mat scaledRGB, preprocessedRGB, scaledD, preprocessedD;
    
    inputsRGB.convertTo(scaledRGB, CV_32F, 1.0f / 255.0f);
    cv::dnn::blobFromImage(scaledRGB, preprocessedRGB);

    inputsD.convertTo(scaledD, CV_32F, 1.0f / 255.0f);
    cv::dnn::blobFromImage(scaledD, preprocessedD);

    size_t inputRGBSize = 1;
    for(int i = 0; (i < inputShapeRGB.size()-1); i++)
    {
        inputRGBSize *= inputShapeRGB[i];
    }

    size_t inputDSize = 1;
    for(int i = 0; (i < inputShapeD.size()-1); i++)
    {
        inputDSize *= inputShapeD[i];
    }

    // Create input tensor object from data values
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    std::vector<float> input_rgb_values(inputRGBSize);
        input_rgb_values.assign(preprocessedRGB.begin<float>(),preprocessedRGB.end<float>());
    Ort::Value input_tensor1 = Ort::Value::CreateTensor<float>(
        memory_info, input_rgb_values.data(), input_rgb_values.size(), inputShapeRGB.data(), inputShapeRGB.size()
    );

    // depth tensor
    std::vector<float> input_d_values(inputDSize);
    input_d_values.assign(preprocessedD.begin<float>(),preprocessedD.end<float>());
    Ort::Value input_tensor2 = Ort::Value::CreateTensor<float>(
        memory_info, input_d_values.data(), input_d_values.size(), inputShapeD.data(), inputShapeD.size()
    );

    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(std::move(input_tensor1));
    input_tensors.push_back(std::move(input_tensor2));

    // Get output node names
    size_t num_output_nodes = session.GetOutputCount();

    // Score the model
    auto output_tensors = session.Run(Ort::RunOptions{nullptr}, inputNames.data(), input_tensors.data(), input_tensors.size(),
                                    outputNames.data(), num_output_nodes);

    // Process output tensor
    float* floatarr = output_tensors[0].GetTensorMutableData<float>();
    size_t tensor_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

    cv::Mat output_mat(480, 640, CV_32FC(40));

    // Reshape and copy data into the OpenCV Mat
    size_t index = 0;
    for (int c = 0; c < 40; ++c) {
        for (int h = 0; h < 480; ++h) {
            for (int w = 0; w < 640; ++w) {
                output_mat.at<cv::Vec<float, 40>>(h, w)[c] = floatarr[index++];
            }
        }
    }

    return output_mat;
}

cv::Mat OnnxPipeline::getMaxChannelIndices(const cv::Mat& multiChannelMat) {
    // Get the dimensions of the input matrix
    int rows = multiChannelMat.rows;
    int cols = multiChannelMat.cols;
    int channels = multiChannelMat.channels();

    // Create an output matrix to store the indices of the max channel
    cv::Mat maxChannelIndices(rows, cols, CV_8UC1);

    // Iterate through each pixel
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            // Find the channel with the maximum value at this pixel
            float maxVal = -std::numeric_limits<float>::max();
            int maxIdx = 0;
            for (int ch = 0; ch < channels; ++ch) {
                float val = multiChannelMat.at<cv::Vec<float, 40>>(row, col)[ch];
                if (val > maxVal) {
                    maxVal = val;
                    maxIdx = ch;
                }
            }
            // Store the index of the channel with the maximum value
            maxChannelIndices.at<uchar>(row, col) = maxIdx;
        }
    }

    return maxChannelIndices;
}

// Function to display the max channel indices using different colors
cv::Mat OnnxPipeline::displayMaxChannelIndices(const cv::Mat& multiChannelMat) {
    // first, get the max channel
    cv::Mat maxChannelIndices = getMaxChannelIndices(multiChannelMat);

    int rows = maxChannelIndices.rows;
    int cols = maxChannelIndices.cols;

    // Create a colored output image
    cv::Mat colorImage(rows, cols, CV_8UC3);

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            uchar idx = maxChannelIndices.at<uchar>(row, col);
            colorImage.at<cv::Vec3b>(row, col) = colorSpace[idx];
        }
    }

    return colorImage;
}

cv::Mat OnnxPipeline::blendImages(const cv::Mat& rawImage, const cv::Mat& maskImage) {
    cv::Mat bgrImage;
    cv::cvtColor(rawImage, bgrImage, cv::COLOR_RGB2BGR);

    cv::Mat rawImageFloat, maskImageFloat;
    bgrImage.convertTo(rawImageFloat, CV_32FC3, 1.0 / 255.0);
    maskImage.convertTo(maskImageFloat, CV_32FC3, 1.0 / 255.0);

    double alpha = 0.4;
    cv::Mat blendedImage;
    cv::addWeighted(rawImageFloat, 1.0 - alpha, maskImageFloat, alpha, 0.0, blendedImage);

    blendedImage.convertTo(blendedImage, CV_8UC3, 255.0);
    return blendedImage;
}


/*int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return -1;
    }

    // Read input image
    cv::Mat image = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        std::cerr << "Could not open or find the image: " << argv[1] << std::endl;
        return -1;
    }

    // Initialize the ONNX pipeline
    OnnxPipeline onnxPipeline("RTFNet.onnx");

    // Run inference
    cv::Mat result = onnxPipeline.forward(image);
    cv::Mat colorImage = onnxPipeline.displayMaxChannelIndices(result);

    // Display the result
    cv::imshow("Inference Result", colorImage);
    cv::waitKey(0);

    return 0;
}*/