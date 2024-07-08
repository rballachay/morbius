#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>

int main() {
    try {
        // Create a pipeline to configure, start and stop the camera
        rs2::pipeline pipe;
        rs2::config cfg;

        // Configure the pipeline to stream depth and color streams at 640x480 resolution
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

        // Start the pipeline with the configured settings
        rs2::pipeline_profile profile = pipe.start(cfg);

        // Get the active device connected to the pipeline
        rs2::device dev = profile.get_device();

        // Get the stereo module (typically contains both left and right cameras)
        rs2::sensor stereo_sensor = dev.query_sensors()[1]; // Assuming the stereo sensor is at index 1

        // Get the intrinsic parameters for the right camera
        rs2::stream_profile right_stream = stereo_sensor.get_stream_profiles()[0]; // Assuming the first stream is the right camera
        rs2_intrinsics right_intrinsics = right_stream.as<rs2::video_stream_profile>().get_intrinsics();

        // Print the intrinsic parameters for the right camera
        std::cout << "Right Camera Intrinsics: " << std::endl;
        std::cout << "  fx: " << right_intrinsics.fx << std::endl;
        std::cout << "  fy: " << right_intrinsics.fy << std::endl;
        std::cout << "  cx: " << right_intrinsics.ppx << std::endl;
        std::cout << "  cy: " << right_intrinsics.ppy << std::endl;
        std::cout << "  Distortion Model: " << right_intrinsics.model << std::endl;
        std::cout << "  Distortion Coefficients: ";
        for (int i = 0; i < 5; ++i) std::cout << right_intrinsics.coeffs[i] << " ";
        std::cout << std::endl;

        // Get the intrinsic parameters for the depth stream
        rs2::stream_profile depth_stream = profile.get_stream(RS2_STREAM_DEPTH);
        rs2_intrinsics depth_intrinsics = depth_stream.as<rs2::video_stream_profile>().get_intrinsics();

        // Print the resolution settings for the depth stream
        std::cout << "Depth Stream Resolution: " << std::endl;
        std::cout << "  Width: " << depth_intrinsics.width << std::endl;
        std::cout << "  Height: " << depth_intrinsics.height << std::endl;

        // Get the extrinsics between the depth and color cameras
        rs2_extrinsics extrinsics = depth_stream.get_extrinsics_to(profile.get_stream(RS2_STREAM_COLOR));

        // The baseline (b) is the translation component along the x-axis
        float baseline = extrinsics.translation[0];
        std::cout << "Baseline (b): " << baseline << " meters" << std::endl;

        // The depth map factor is typically 1000.0 for converting millimeters to meters
        float depth_map_factor = 1000.0;
        std::cout << "Depth Map Factor: " << depth_map_factor << std::endl;

        // The close/far threshold (ThDepth) is usually an application-specific value
        // For example, you can set it based on the depth sensor's capabilities or your application's requirements
        float th_depth_close = 0.1; // Minimum threshold in meters
        float th_depth_far = 4.0;   // Maximum threshold in meters
        std::cout << "Close Threshold (ThDepth Close): " << th_depth_close << " meters" << std::endl;
        std::cout << "Far Threshold (ThDepth Far): " << th_depth_far << " meters" << std::endl;

    } catch (const rs2::error & e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n"
                  << e.what() << std::endl;
    } catch (const std::exception & e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
