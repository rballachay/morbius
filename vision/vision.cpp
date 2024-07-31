#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include "artificialFields.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>

#include <System.h>

#define AVG_FRAMES 10

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

void interpolateData(const std::vector<double> &vBase_times,
                     std::vector<rs2_vector> &vInterp_data, std::vector<double> &vInterp_times,
                     const rs2_vector &prev_data, const double &prev_time);

rs2_vector interpolateMeasure(const double target_time,
                              const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./mono_inertial_realsense_D415 path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else
        selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
     for (rs2::sensor sensor : sensors) {
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            std::string sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);

            if (sensor.is<rs2::depth_sensor>()) {
                // Depth sensor (index 1 in your original logic)
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                //sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 50000);
                //sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
            } else if (sensor.is<rs2::color_sensor>()) {
                // Color sensor (index 2 in your original logic)
                sensor.set_option(RS2_OPTION_EXPOSURE, 80.f);
            }

            // Optionally print sensor information
            std::cout << "Sensor Name: " << sensor_name << std::endl;
            //get_sensor_option(sensor); // Implement your function to get sensor options
        }
    }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_RGB8, 15);

    // Depth stream
    cfg.enable_stream(RS2_STREAM_DEPTH,640, 480, RS2_FORMAT_Z16, 15);
    
    cv::Mat imCV, depthCV;
    int width_img, height_img;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames

    // start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);

    // Align depth and RGB frames
    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);
    rs2::frameset fsSLAM;

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);

    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " <<
    intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im, depth;

    double t_resize = 0.f;
    double t_track = 0.f;
    rs2::frameset fs;

    // all the config from the planeSegment file
    float depthScale = 1000;
    int frameCount=0;
    std::vector<cv::Mat> depths(AVG_FRAMES);
    std::vector<cv::Mat> colorImages(AVG_FRAMES);

    std::queue<cv::Mat> frameQueue;
    std::mutex queueMutex;
    std::condition_variable frameAvailable;

    auto run = [&] () {
        while (!SLAM.isShutDown())
        {
            fs = pipe.wait_for_frames();;

            // Perform alignment here
            auto processed = align.process(fs);

            // Trying to get both other and aligned depth frames
            rs2::video_frame color_frame = processed.first(align_to);
            rs2::depth_frame depth_frame = processed.get_depth_frame();

            im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
            depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

            if(imageScale != 1.f)
            {
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
                cv::resize(depth, depth, cv::Size(width, height));

            }
            // Pass the image to the SLAM system
            Sophus::SE3f se3_transform = SLAM.TrackRGBD(im, depth, timestamp); //, vImuMeas); depthCV

            // point in the global reference frame
            Eigen::Vector3f point_global(0.0f, 0.0f, -0.5f);

            // we want to know where we are navigating to in the local reference frame
            Eigen::Vector3f point_local = se3_transform.inverse() * point_global;

            std::cout << "Point in the local reference frame: " << point_local.transpose() << std::endl;

            // all the processing for the planeSegment
            colorImages[frameCount] = im.clone();
        	depths[frameCount] = depth.clone();
			frameCount++;

            if(frameCount == AVG_FRAMES)
        	{
                PlaneDetection plane_detection;

				cv::Mat avgColor = computeAverage(colorImages);
				cv::Mat avgDepth = computeAverage(depths);
				
				plane_detection.readDepthImage(avgDepth);
				plane_detection.readColorImage(avgColor);

				plane_detection.runPlaneDetection();

				plane_detection.plane_filter.publicRefineDetails(&plane_detection.plane_vertices_, nullptr, &plane_detection.seg_img_);
				
				Surfaces surfaces(plane_detection);

				cv::Mat floorHeat = avgColor.clone();

				if (surfaces.groundIdx!=-1){
					Plane plane = computePlaneEq(surfaces.planes, surfaces.groundIdx);
					std::vector<Eigen::Vector3d> projectedVertices = projectOnPlane(surfaces.vertices, plane);

					pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud = makeVoxelCloud(projectedVertices, plane_detection.plane_vertices_);

					floorHeat = drawFloorHeatMap(surfaces.vertices, 
						plane_detection.plane_vertices_, surfaces.groundIdx, voxelCloud, plane, avgColor);
				}
                
                {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    frameQueue.push(floorHeat);
                }
                frameAvailable.notify_one();
                
				frameCount=0;
        	}
        }
    };
    // create a background thread to run slam on, and run the slam viewer in 
    // the main thread
    std::thread SlamThread(run);
    SlamThread.detach();
    SLAM.getViewer()->Run();

    cout << "System shutdown!\n";
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}


bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}