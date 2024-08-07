#include "realsense.hpp"
#include <librealsense2/rs.hpp>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs_advanced_mode.hpp> 


void RealSense::configureCameraSettings() {    
    std::vector<rs2::sensor> sensors = profile.get_device().query_sensors();
    for (rs2::sensor& sensor : sensors) {
        if (sensor.get_stream_profiles().front().stream_type() == RS2_STREAM_COLOR) {
            color_sensor = sensor;
            break;
        }
    }
    // Get the first device
    rs2::device dev = profile.get_device();
    std::string dev_serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    std::cout << "Device found: " << dev_serial_number << std::endl;

    // Access advanced mode interface
    auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
    if (!advanced_mode_dev) {
        std::cerr << "Failed to get advanced mode interface!" << std::endl;
        return;
    }

    // Load and configure .json file
    std::ifstream t("configs/HighDensityPreset.json");
    if (!t.is_open()) {
        std::cerr << "Failed to open configuration file!" << std::endl;
        return;
    }

    std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    if (str.empty()) {
        std::cerr << "Configuration file is empty!" << std::endl;
        return;
    }

    advanced_mode_dev.load_json(str);

    rs2::depth_sensor sensor = dev.first<rs2::depth_sensor>();
    depthScale = sensor.get_depth_scale();

    // if there are no exposures set, turn on auto exposure
    if (exposures.empty()){
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    }
}

void RealSense::warmUpPipeline() {
    for (int i = 0; i < 50; i++) {
        rs2::frameset frames = pipeline.wait_for_frames();
    }
}

RealSense::RealSense(const std::vector<int>& exposures)
    : exposures(exposures) {
}

void RealSense::startPipeline() {
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
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
    int i = 0; 
    rs2::align align_to(RS2_STREAM_COLOR);
    while (true) {
        // set exposure if we want custom dynamic exposure
        if (!exposures.empty()){
            color_sensor.set_option(RS2_OPTION_EXPOSURE, exposures[i]);
            i++;
            if (i==exposures.size())i=0; //reset
        }
        rs2::frameset frames = pipeline.wait_for_frames();
        auto aligned_frames = align_to.process(frames);
        frameHandler(aligned_frames);
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
    cv::Mat depth_mat(cv::Size(640, 480), CV_16UC1, (void*)depth_processed.get_data(), cv::Mat::AUTO_STEP);

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
    thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 3.0f);

    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 5);
    spat_filter.set_option(RS2_OPTION_HOLES_FILL, 5);
    
    rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);  // Set alpha parameter
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 80.0); // Set delta parameter
    
    rs2::hole_filling_filter hole_filling_filter;   // Hole filling
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 2);  // Hole filling parameter - 2 is near pixels

    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    // Initialize a vector that holds filters and their options
    std::vector<filter_options> filters;

    // The following order of emplacement will dictate the orders in which filters are applied
    //filters.emplace_back("Decimate", dec_filter);
    filters.emplace_back("Threshold", thr_filter);
    //filters.emplace_back("Disparity", depth_to_disparity);
    filters.emplace_back("Spatial", spat_filter);
    filters.emplace_back("Temporal", temp_filter);
    //filters.emplace_back("Depth",disparity_to_depth);
    filters.emplace_back("Hole filling", hole_filling_filter);

    for (auto&& filter : filters) {
        filtered = filter.filter.process(filtered);
    }

    return filtered;
}

// Function to apply the non-zero median filter
template<typename T>
cv::Mat applyNonZeroMedianFilter(const cv::Mat& image, int factor) {
    cv::Mat filteredImage = cv::Mat::zeros(image.rows / factor, image.cols / factor, image.type());

    for (int i = 0; i < image.rows; i += factor) {
        for (int j = 0; j < image.cols; j += factor) {
            std::vector<T> values;

            for (int k = 0; k < factor; ++k) {
                for (int l = 0; l < factor; ++l) {
                    int x = i + k;
                    int y = j + l;
                    if (x < image.rows && y < image.cols) {
                        T pixelValue = image.at<T>(x, y);
                        // Check if the pixel is non-zero
                        if (cv::sum(pixelValue)[0] > 0) {
                            values.push_back(pixelValue);
                        }
                    }
                }
            }

            if (!values.empty()) {
                // Sort the values to find the median
                std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end(),
                    [](const T& a, const T& b) {
                        return cv::sum(a)[0] < cv::sum(b)[0];
                    });
                filteredImage.at<T>(i / factor, j / factor) = values[values.size() / 2];
            }
        }
    }

    cv::copyMakeBorder(filteredImage, filteredImage, 480 - filteredImage.rows, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    return filteredImage;
}


// Explicit instantiation for the types you need
template cv::Mat applyNonZeroMedianFilter<uint16_t>(const cv::Mat& image, int factor);
template cv::Mat applyNonZeroMedianFilter<cv::Vec3b>(const cv::Mat& image, int factor);

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