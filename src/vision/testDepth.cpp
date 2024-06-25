#include "realsense.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/rs.hpp>

class filter_options {
public:
    filter_options(const std::string& name, rs2::filter& filter);

    std::string filter_name;      // Friendly name of the filter
    rs2::filter& filter;          // The filter in use
};
filter_options::filter_options(const std::string& name, rs2::filter& filter)
    : filter_name(name), filter(filter) {}

rs2::frame filterDepth(rs2::frame& depth_frame){
    rs2::frame filtered = depth_frame; 

    //rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    // Initialize a vector that holds filters and their options
    std::vector<filter_options> filters;

    // The following order of emplacement will dictate the orders in which filters are applied
    //filters.emplace_back("Decimate", dec_filter);
    filters.emplace_back("Threshold", thr_filter);
    filters.emplace_back("Disparity", depth_to_disparity);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);
    filters.emplace_back("Depth",disparity_to_depth);

    for (auto&& filter : filters) {
        filtered = filter.filter.process(filtered);
    }

    return filtered;
}


int main() {
    try {
        RealSense realsense;
        realsense.startPipeline();

        realsense.captureFrames([](const rs2::frameset& frames) {
                // Retrieve depth and color frames
            rs2::depth_frame depth = frames.get_depth_frame();

            rs2::frame filteredDepth = filterDepth(depth);

            // Convert depth frame to CV_16U Mat
            cv::Mat depth_mat(cv::Size(filteredDepth.as<rs2::video_frame>().get_width(), filteredDepth.as<rs2::video_frame>().get_height()), 
                      CV_16U, (void*)filteredDepth.get_data(), cv::Mat::AUTO_STEP);

            // Display the frame
            cv::imshow("Processed Frame", depth_mat);
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