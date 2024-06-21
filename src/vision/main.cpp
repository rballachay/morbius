#include "realsense.hpp"
#include "onnxpipeline.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/rs.hpp>

int main() {
    try {
        RealSense realsense;
        realsense.startPipeline();

        // Initialize the ONNX pipeline
        OnnxPipeline onnxPipeline("RTFNet.onnx");

        realsense.captureFrames([&onnxPipeline](const rs2::frameset& frames) {

            // return a {480, 640, 4} RGB-D cv::Mat
            cv::Mat processedFrame = depthMatFrameProcess(frames);

            cv::Mat result = onnxPipeline.forward(processedFrame);
            cv::Mat colorImage = onnxPipeline.displayMaxChannelIndices(result);
            
            // Display the frame
            cv::imshow("Processed Frame", colorImage);
            if (cv::waitKey(1) == 27) { // Exit on ESC key
                std::exit(0);
            }
        });
    } catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n"
                  << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception & e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}