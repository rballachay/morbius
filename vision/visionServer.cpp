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
#include <opencv2/imgcodecs.hpp>
#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"
#include "src/rgbdSeg/artificialFields.hpp"
#include "src/rgbdSeg/groundVectors.hpp"
#include <System.h>
#include <pistache/endpoint.h>
#include <pistache/http.h>
#include <pistache/peer.h>
#include <json/json.h>

// rotation is very very bad for an un-mapped system. It is important to do all the mapping 
// and then come back to the environment and run the localization. For more information 
// about this, go see this link: https://github.com/raulmur/ORB_SLAM2/issues/194#issuecomment-259948302

#define AVG_FRAMES 10

using namespace std;
using namespace Pistache;

// this is the current destination of the robot
pcl::PointXYZ destination(0, 0, 0);

// local destination - where the next robot step should take it
pcl::PointXYZ local_destination(0, 0, 0);

// this is the current location of the robot
pcl::PointXYZ location(0, 0, 0); 

// max traversible distance calculated with heading vectors
float max_distance = 0.0; 

std::vector<cv::Mat> floorHeatMaps;

atomic<bool> b_continue_session(true);
condition_variable cond_v;
sigset_t signals;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
    cond_v.notify_all();  // Notify any waiting threads
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

struct RequestHandler : public Http::Handler {
    HTTP_PROTOTYPE(RequestHandler)

    void onRequest(const Http::Request& request, Http::ResponseWriter writer) override {
        if (request.method() == Http::Method::Post) {
            // Extract the request body
            auto body = request.body();
            
            // Parse the JSON body
            Json::CharReaderBuilder readerBuilder;
            Json::Value jsonData;
            std::string errs;

            std::istringstream s(body);
            if (Json::parseFromStream(readerBuilder, s, &jsonData, &errs)) {
                // Extract x, y, z from the JSON
                if (jsonData.isMember("x") && jsonData.isMember("y") && jsonData.isMember("z")) {
                    float x = jsonData["x"].asFloat();
                    float y = jsonData["y"].asFloat();
                    float z = jsonData["z"].asFloat();

                    // Update destination
                    destination.x = x;
                    destination.y = y;
                    destination.z = z;

                    writer.send(Http::Code::Ok, "Updated destination successfully\n");
                } else {
                    writer.send(Http::Code::Bad_Request, "Missing parameters in JSON body\n");
                }
            } else {
                writer.send(Http::Code::Bad_Request, "Invalid JSON format\n");
            }
        } else if (request.method() == Http::Method::Get) {
            // Create JSON response with source and destination
            Json::Value responseData;
            responseData["location"]["x"] = location.x;
            responseData["location"]["y"] = location.y;
            responseData["location"]["z"] = location.z;
            responseData["local_destination"]["x"] = local_destination.x;
            responseData["local_destination"]["y"] = local_destination.y;
            responseData["local_destination"]["z"] = local_destination.z;
            responseData["max_distance"] = max_distance;

            // Serialize JSON to string
            Json::StreamWriterBuilder writerBuilder;
            std::string responseString = Json::writeString(writerBuilder, responseData);

            // Send JSON response
            writer.send(Http::Code::Ok, responseString, MIME(Application, Json));
        } else {
            writer.send(Http::Code::Method_Not_Allowed, "Only GET and POST methods are allowed\n");
        }
    }
};

int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./mono_inertial_realsense_D415 path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    // load the step size from the json config
    Json::Value config;
    std::ifstream config_file("config.json", std::ifstream::binary);
    config_file >> config;
    int step_size = config["STEP_SIZE"].asInt();
    int port = config["PORT"].asInt();


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

    sigemptyset(&signals);
    sigaddset(&signals, SIGINT);
    
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

    auto run = [&] () {
        while (!SLAM.isShutDown())
        {
            if (!b_continue_session){
                SLAM.Shutdown();
                break;
            }
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

            // all the processing for the planeSegment
            colorImages[frameCount] = im.clone();
        	depths[frameCount] = depth.clone();
			frameCount++;

            if(frameCount == AVG_FRAMES)
        	{
                PlaneDetection plane_detection;

				cv::Mat avgColor_bgr = computeAverage(colorImages);

                // this algorithm is developed on BGR images
                cv::Mat avgColor;
                cv::cvtColor(avgColor_bgr, avgColor, cv::COLOR_RGB2BGR);

				cv::Mat avgDepth = computeAverage(depths);
				
				plane_detection.readDepthImage(avgDepth);
				plane_detection.readColorImage(avgColor);

				plane_detection.runPlaneDetection();
				plane_detection.plane_filter.publicRefineDetails(&plane_detection.plane_vertices_, nullptr, &plane_detection.seg_img_);
				
				Surfaces surfaces(plane_detection);

                cv::Mat floorHeat;
				floorHeat = avgColor.clone();

				cv::Mat groundVecs;
				groundVecs = avgColor.clone();

				cv::Mat mask = plane_detection.seg_img_;

				float max_distance = 0;

				if (surfaces.groundIdx!=-1){
                    mask = makeMask(plane_detection, surfaces.groundIdx);

					Plane plane = computePlaneEq(surfaces.planes, surfaces.groundIdx);
					std::vector<Eigen::Vector3d> projectedVertices = projectOnPlane(surfaces.vertices, plane);

					pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud = makeVoxelCloud(projectedVertices, plane_detection.plane_vertices_);

                    Eigen::Vector3f translation = se3_transform.translation();
                    location.x = translation.x();
                    location.y = translation.y();
                    location.z = translation.z();

					Forces forces = resultantForces(voxelCloud, location, destination);
        
                    local_destination = calculateStep(location, forces, step_size);

                    cv::Mat floorHeat = drawFloorHeatMap(surfaces.vertices, 
						plane_detection.plane_vertices_, surfaces.groundIdx, voxelCloud, plane, avgColor);

                    floorHeatMaps.push_back(floorHeat);

                    std::tie(groundVecs, max_distance) = drawDistanceVectors(avgColor, plane_detection, surfaces, mask);
				}
                
				frameCount=0;
        	}
        }
    };

    std::thread slam_thread(run);
    slam_thread.detach();

    Pistache::Address addr(Pistache::Ipv4::any(), Pistache::Port(port));
    auto opts = Pistache::Http::Endpoint::options().threads(1);
    Pistache::Http::Endpoint server(addr);
    server.init(opts);
    server.setHandler(Pistache::Http::make_handler<RequestHandler>());
    server.serveThreaded();

    bool terminate = false;
    while (!terminate) {
        int number = 0;
        int status = sigwait(&signals, &number);
        if (status != 0) {
            break;
        }

        switch (number) {
            case SIGINT:
                std::cout << "\rCaught signal " << number << " (SIGINT)." << std::endl;
                terminate = true;
                break;
            default:
                std::cout << "\rCaught signal " << number << "." << std::endl;
                break;
        }

    }

    std::cout << "Shutting down the HTTP server." << std::endl;
    server.shutdown();

    cout << "System shutdown!\n";

    // Save all collected floor heatmaps to the results/ folder
    if (!std::filesystem::exists("results")) {
        std::filesystem::create_directory("results");
    }

    int valInx = 0;
    for (const auto& mat : floorHeatMaps) {
        std::ostringstream filename;
        filename << "results/floorHeat_" << valInx++ << ".png";
        cv::imwrite(filename.str(), mat);
    }

    cout << "Saved all floor heatmaps to results/ directory." << endl;

    return 0;
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