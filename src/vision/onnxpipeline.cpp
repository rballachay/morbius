#include "onnxpipeline.hpp"
#include <vector>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <thread>

OnnxPipeline::OnnxPipeline(const char* modelPath) 
    : modelPath(modelPath), env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference"), session(nullptr),
    sessionOptions(nullptr), inputShape({1, 4, 480, 640}), inputNames({ "l_input_" }), outputNames({ "deconv5_1"}){
    
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(1);
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Create an ONNX Runtime session
    session = Ort::Session(env, modelPath, sessionOptions);

    // Get input node names and shapes
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_input_nodes = session.GetInputCount();

}

cv::Mat OnnxPipeline::forward(cv::Mat inputs){

    cv::Mat scaledImage, preprocessedImage;
    inputs.convertTo(scaledImage, CV_32F, 1.0f / 255.0f);
    cv::dnn::blobFromImage(scaledImage, preprocessedImage);

    size_t inputTensorSize = 1;
    for(int i = 0; (i < inputShape.size()-1); i++)
    {
        inputTensorSize *= inputShape[i];
    }

    std::vector<float> input_tensor_values(inputTensorSize);
        input_tensor_values.assign(preprocessedImage.begin<float>(),preprocessedImage.end<float>());
    
    // Create input tensor object from data values
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_tensor_values.data(), input_tensor_values.size(), inputShape.data(), inputShape.size()
    );

    // Get tensor type and shape information
    Ort::TensorTypeAndShapeInfo type_and_shape_info = input_tensor.GetTensorTypeAndShapeInfo();

    // Get output node names
    size_t num_output_nodes = session.GetOutputCount();

    // Score the model
    auto output_tensors = session.Run(Ort::RunOptions{nullptr}, inputNames.data(), &input_tensor, 1,
                                    outputNames.data(), num_output_nodes);


    // Process output tensor
    float* floatarr = output_tensors[0].GetTensorMutableData<float>();
    size_t tensor_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    
    cv::Mat output_mat(480, 640, CV_32FC(9));

    // Reshape and copy data into the OpenCV Mat
    size_t index = 0;
    for (int c = 0; c < 9; ++c) {
        for (int h = 0; h < 480; ++h) {
            for (int w = 0; w < 640; ++w) {
                output_mat.at<cv::Vec<float, 9>>(h, w)[c] = floatarr[index++];
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
                float val = multiChannelMat.at<cv::Vec<float, 9>>(row, col)[ch];
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

    // Define colors for each channel index
    std::vector<cv::Vec3b> colors = {
        {0, 0, 255},    // Red for channel 0
        {255, 0, 0},    // Blue for channel 1
        {0, 255, 0},    // Green for channel 2
        {255, 255, 0},  // Cyan for channel 3
        {255, 0, 255},  // Magenta for channel 4
        {0, 255, 255},  // Yellow for channel 5
        {128, 0, 128},  // Purple for channel 6
        {128, 128, 0},  // Olive for channel 7
        {0, 128, 128}   // Teal for channel 8
    };

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            uchar idx = maxChannelIndices.at<uchar>(row, col);
            colorImage.at<cv::Vec3b>(row, col) = colors[idx];
        }
    }

    return colorImage;
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