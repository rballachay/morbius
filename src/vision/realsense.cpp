#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <functional>
#include <opencv2/opencv.hpp>

class RealSense {
private:
    rs2::pipeline pipeline;
    rs2::config config;
    rs2::pipeline_profile profile;

    void configureCameraSettings() {    
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

    void warmUpPipeline() {
        for (int i = 0; i < 50; i++) {
            rs2::frameset frames = pipeline.wait_for_frames();
        }
    }

public:
    RealSense() = default;

    void startPipeline() {
        config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        profile = pipeline.start(config);

        configureCameraSettings();
        warmUpPipeline();
    }

    rs2::pipeline_profile getPipelineProfile() const {
        return profile;
    }

    rs2::pipeline& getPipeline() {
        return pipeline;
    }

    void captureFrames(const std::function<void(const rs2::frameset&)>& frameHandler) {
        while (true) {
            rs2::frameset frames = pipeline.wait_for_frames();
            frameHandler(frames);
        }
    }
};

cv::Mat depthMatFrameProcess(const rs2::frameset& frames) {
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame rgb_frame = frames.get_color_frame();

    cv::Mat depth_mat(cv::Size(640, 480), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);

    cv::Mat depth_mat_8uc1;
    cv::convertScaleAbs(depth_mat, depth_mat_8uc1, 255.0f / 65535.0f);

    std::vector<cv::Mat> channels_vec;
    channels_vec.push_back(depth_mat_8uc1);
    
    cv::Mat color_channels[3];
    cv::split(color_mat, color_channels);
    channels_vec.push_back(color_channels[0]);
    channels_vec.push_back(color_channels[1]);
    channels_vec.push_back(color_channels[2]);

    cv::Mat stacked_mat;
    cv::merge(channels_vec, stacked_mat);
    
    return stacked_mat;
}

int main() {
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
}