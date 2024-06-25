#include "realsense.hpp"
#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>


void RealSense::configureCameraSettings() {    
    std::vector<rs2::sensor> sensors = profile.get_device().query_sensors();
    rs2::sensor color_sensor;
    for (rs2::sensor& sensor : sensors) {
        if (sensor.get_stream_profiles().front().stream_type() == RS2_STREAM_COLOR) {
            color_sensor = sensor;
            break;
        }
    }

    if (color_sensor) {
        if (color_sensor.supports(RS2_OPTION_BRIGHTNESS)) {
            color_sensor.set_option(RS2_OPTION_BRIGHTNESS, 30);
            std::cout << "Brightness set to 30." << std::endl;
        }
        if (color_sensor.supports(RS2_OPTION_GAMMA)) {
            color_sensor.set_option(RS2_OPTION_GAMMA, 300);
            std::cout << "Gamma set to 300." << std::endl;
        }
        if (color_sensor.supports(RS2_OPTION_SHARPNESS)) {
            color_sensor.set_option(RS2_OPTION_SHARPNESS, 50);
            std::cout << "Sharpness set to 50." << std::endl;
        }
        if (color_sensor.supports(RS2_OPTION_SATURATION)) {
            color_sensor.set_option(RS2_OPTION_SATURATION, 32);
            std::cout << "Saturation set to 32." << std::endl;
        }
    } else {
        std::cerr << "No color sensor found!" << std::endl;
    }
}

void RealSense::warmUpPipeline() {
    for (int i = 0; i < 50; i++) {
        rs2::frameset frames = pipeline.wait_for_frames();
    }
}

RealSense::RealSense() = default;

void RealSense::startPipeline() {
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    profile = pipeline.start(config);

    configureCameraSettings();
    warmUpPipeline();
}

rs2::pipeline_profile RealSense::getPipelineProfile() const {
    return profile;
}

rs2::pipeline& RealSense::getPipeline() {
    return pipeline;
}

void RealSense::captureFrames(const std::function<void(const rs2::frameset&)>& frameHandler) {
    while (true) {
        rs2::frameset frames = pipeline.wait_for_frames();
        frameHandler(frames);
    }
}

cv::Mat minMaxScale(const cv::Mat& inputImage, int cvType) {
    cv::Mat scaledImage;

    // Normalize each channel independently to range [0, 255]
    cv::normalize(inputImage, scaledImage, 0, 255, cv::NORM_MINMAX, cvType);

    return scaledImage;
}

std::pair<cv::Mat, cv::Mat> depthMatFrameProcess(const rs2::frameset& frames) {
    // Retrieve depth and color frames
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame rgb_frame = frames.get_color_frame();

    rs2::frame depth_processed = preprocessDepth(depth);

    // Convert depth frame to CV_16U Mat
    cv::Mat depth_mat(cv::Size(640, 480), CV_16U, (void*)depth_processed.get_data(), cv::Mat::AUTO_STEP);

    // Convert color frame to CV_8UC3 Mat
    cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);

    // Convert depth_mat to CV_8UC1 for visualization (optional)
    cv::Mat depth_mat_8uc1;
    cv::convertScaleAbs(depth_mat, depth_mat_8uc1, 255.0f / 65535.0f);

    return std::make_pair(color_mat, depth_mat_8uc1);
}

std::pair<cv::Mat, cv::Mat> postProcessFrames(cv::Mat& color_mat, cv::Mat& depth_mat){
    // Optionally perform min-max scaling or normalization
    cv::Mat colorScaled = minMaxScale(color_mat, CV_8UC3); // Implement minMaxScale function as needed
    cv::Mat depthScaled = minMaxScale(depth_mat, CV_8UC1); // Implement minMaxScale function as needed

    cv::Mat normalizedColor = normalizeRGB(colorScaled);
    cv::Mat normalizedDepth = normalizeDepth(depthScaled);

    return std::make_pair(normalizedColor, normalizedDepth);
}

cv::Mat normalizeRGB(cv::Mat& rgbImage){

    // Convert the image to float
    cv::Mat floatImage;
    rgbImage.convertTo(floatImage, CV_32F); // Scale to [0, 1] range

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

    return normalizedImage;
}

cv::Mat normalizeDepth(cv::Mat& depthImage){
    // Convert the image to float
    cv::Mat floatDepth;
    depthImage.convertTo(floatDepth, CV_32F); // Scale to [0, 1] range
    floatDepth -= 0.5;  // Subtract mean
    floatDepth /= 0.25; // Divide by standard deviation
    return floatDepth;
}

class filter_options {
public:
    filter_options(const std::string& name, rs2::filter& filter);

    std::string filter_name;      // Friendly name of the filter
    rs2::filter& filter;          // The filter in use
};
filter_options::filter_options(const std::string& name, rs2::filter& filter)
    : filter_name(name), filter(filter) {}

rs2::frame preprocessDepth(rs2::frame& depth_frame){
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

/*int main() {
    try {
        RealSense realsense;
        realsense.startPipeline();

        realsense.captureFrames([](const rs2::frameset& frames) {

            // return a {480, 640, 4} RGB-D cv::Mat
            cv::Mat processedFrame = depthMatFrameProcess(frames);
            
            // Display the frame
            cv::imshow("Processed Frame", processedFrame);
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
}*/