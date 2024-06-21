// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <vector>
#include <onnxruntime/onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <thread>



cv::Mat getMaxChannelIndices(const cv::Mat& multiChannelMat) {
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
cv::Mat displayMaxChannelIndices(const cv::Mat& maxChannelIndices) {
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

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char * argv[]) try
{

    // Initialize the ONNX Runtime environment
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXInference");

    // Create a session options object
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);

    // Set graph optimization level
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Specify the model path
    const char* model_path = "RTFNet.onnx";

    // Create an ONNX Runtime session
    Ort::Session session(env, model_path, session_options);

    // Get input node names and shapes
    Ort::AllocatorWithDefaultOptions allocator;
    size_t num_input_nodes = session.GetInputCount();
    std::vector<const char*> input_node_names = { "l_input_" }; // Input node names

    // Example input tensor
    std::vector<int64_t> input_shape = {1, 4, 480, 640}; // Example shape: {batch_size, channels, height, width}
    
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30); // Example configuration

    // Configure and start the pipeline
    rs2::pipeline_profile profile = p.start(cfg);

    // Retrieve the sensor associated with the color stream
    auto sensors = profile.get_device().query_sensors();
    rs2::sensor color_sensor;
    for (auto& sensor : sensors) {
        if (sensor.get_stream_profiles().front().stream_type() == RS2_STREAM_COLOR) {
            color_sensor = sensor;
            break;
        }
    }
    int warm_up=0;

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // need to wait a few frames in order for the auto-exposure to warm up
        if (warm_up<50){
            warm_up++;
            continue;
        }

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::video_frame rgb_frame = frames.get_color_frame();

        cv::Mat depth_mat(cv::Size(640, 480), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
        
        //cv::imshow("Color Image", depth_mat);
        //cv::waitKey(0);

        cv::Mat depth_mat_8uc1;
        cv::convertScaleAbs(depth_mat, depth_mat_8uc1, 4.0*255.0f / 65535.0f); // Scale to 8-bit range

        double minVal, maxVal;
        cv::minMaxIdx(depth_mat_8uc1, &minVal, &maxVal); // Find the min and max values
        std::cout << "Min value: " << minVal << ", Max value: " << maxVal << std::endl;

        // Create a vector of Mat objects
        cv::Mat stacked_mat;
        std::vector<cv::Mat> channels_vec;
        channels_vec.push_back(depth_mat_8uc1);
        cv::Mat color_channels[3];
        cv::split(color_mat, color_channels);
        channels_vec.push_back(color_channels[0]);
        channels_vec.push_back(color_channels[1]);
        channels_vec.push_back(color_channels[2]);

        cv::merge(channels_vec, stacked_mat);

        // Print shape information
        std::cout << "Shape of stacked_mat: " << stacked_mat.size << std::endl;
        std::cout << "Channels of stacked_mat: " << stacked_mat.channels() << std::endl;

        cv::Mat scaledImage, preprocessedImage;
        stacked_mat.convertTo(scaledImage, CV_32F, 1.0f / 255.0f);
        cv::dnn::blobFromImage(scaledImage, preprocessedImage);
        size_t inputTensorSize = 1;
        for(int i = 0; (i < input_shape.size()-1); i++)
        {
            inputTensorSize *= input_shape[i];
        }
        std::vector<float> input_tensor_values(inputTensorSize);
            input_tensor_values.assign(preprocessedImage.begin<float>(),preprocessedImage.end<float>());
        
        // Create input tensor object from data values
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size()
        );

        // Get tensor type and shape information
        Ort::TensorTypeAndShapeInfo type_and_shape_info = input_tensor.GetTensorTypeAndShapeInfo();

        // Get the shape
        std::vector<int64_t> shape = type_and_shape_info.GetShape();

        // Print the shape
        std::cout << "Tensor shape: [";
        for (size_t i = 0; i < shape.size(); ++i) {
            std::cout << shape[i];
            if (i < shape.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;

        // Get output node names
        size_t num_output_nodes = session.GetOutputCount();
        std::vector<const char*> output_node_names  = { "deconv5_1" };

        // Score the model
        auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1,
                                        output_node_names.data(), num_output_nodes);


        // Process output tensor
        float* floatarr = output_tensors[0].GetTensorMutableData<float>();
        size_t tensor_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

        std::cout << tensor_size << std::endl;
        
        // Initialize OpenCV Mat with dimensions and type (adjust type as per your data)
        cv::Mat output_mat(480, 640, CV_32FC(9)); // CV_32FC(num_channels) for multi-channel float

        // Reshape and copy data into the OpenCV Mat
        size_t index = 0;
        for (int c = 0; c < 9; ++c) {
            for (int h = 0; h < 480; ++h) {
                for (int w = 0; w < 640; ++w) {
                    output_mat.at<cv::Vec<float, 9>>(h, w)[c] = floatarr[index++];
                }
            }
        }
        
        cv::Mat results_arr = getMaxChannelIndices(output_mat);

        cv::Mat coloredOutput = displayMaxChannelIndices(results_arr);

        // Display the colored output image
        cv::imshow("Max Channel Indices Color Map", coloredOutput);
        cv::waitKey(0);

        // Display each channel separately
        /*for (int i = 0; i < 9; ++i) {
            std::string window_name = "Channel " + std::to_string(i);

            double minVal, maxVal;
            cv::Point minLoc, maxLoc;

            // Find the minimum and maximum values and their locations
            cv::minMaxLoc( channels[i], &minVal, &maxVal, &minLoc, &maxLoc);

            std::cout << "Minimum value: " << minVal << " at " << minLoc << std::endl;
            std::cout << "Maximum value: " << maxVal << " at " << maxLoc << std::endl;


            cv::imshow(window_name, channels[i]);
            cv::waitKey(0);
        }*/

    }
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}