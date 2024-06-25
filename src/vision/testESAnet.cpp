#include "onnxpipeline.hpp"
#include "realsense.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

/*
Test the ESAnet.onnx model on the sample images. Helps to get a baseline
for what preprocessing needs to be copied over from that repo.
*/
int main() {
    // Initialize the ONNX pipeline
    OnnxPipeline onnxPipeline("ESAnet.onnx");

    cv::Mat rgbImage, rgbScaled, depthScaled;
    cv::Mat image = cv::imread("../../submodules/ESANet/samples/sample_rgb.png");
    cv::cvtColor(image, rgbImage, cv::COLOR_BGR2RGB);
    cv::resize(rgbImage, rgbScaled, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

    // Convert the image to float
    cv::Mat floatImage;
    rgbScaled.convertTo(floatImage, CV_32F); // Scale to [0, 1] range

    // Split the image into its three channels
    std::vector<cv::Mat> channels(3);
    cv::split(floatImage, channels);

    std::vector<float> means = {0.485, 0.456, 0.406};
    std::vector<float> stdevs = {0.229, 0.224, 0.225};

    // Normalize each channel separately
    for (int i = 0; i < 3; ++i) {
        channels[i] -= means[i];  // Subtract mean
        channels[i] /= stdevs[i]; // Divide by standard deviation
    }

    // Merge the normalized channels back into a single image
    cv::Mat normalizedImage;
    cv::merge(channels, normalizedImage);

    // Convert the normalized image back to an 8-bit format for display
    double min, max;
    cv::minMaxLoc(normalizedImage, &min, &max); // Find min and max values for normalization
    normalizedImage.convertTo(normalizedImage, CV_8U, 255.0 / (max - min), -255.0 * min / (max - min));

    cv::Mat depthImage = cv::imread("../../submodules/ESANet/samples/sample_depth.png", cv::IMREAD_GRAYSCALE);
    cv::resize(depthImage, depthScaled,  cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);

    // Convert the image to float
    cv::Mat floatDepth;
    depthScaled.convertTo(floatDepth, CV_32F); // Scale to [0, 1] range

    //cv::Scalar mean, stddev;
    //cv::meanStdDev(floatDepth, mean, stddev);
    floatDepth -= 0.5;  // Subtract mean
    floatDepth /= 0.25; // Divide by standard deviation

    cv::Mat result = onnxPipeline.forward(normalizedImage, floatDepth);

    cv::Mat colorImage = onnxPipeline.displayMaxChannelIndices(result);

    cv::Mat blendedImage= onnxPipeline.blendImages(rgbScaled, colorImage);
    
    // Display the frame
    cv::imshow("Processed Frame", blendedImage);
    cv::waitKey(0); // Wait indefinitely until a key is pressed
    return EXIT_SUCCESS;
}