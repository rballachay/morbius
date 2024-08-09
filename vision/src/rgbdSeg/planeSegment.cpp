#include "realsense.hpp"
#include "plane_detection.h"
#include "groundVectors.hpp"
#include "artificialFields.hpp"
#include <Eigen/Eigen>

// can be changed to an arg later
#define AVG_FRAMES 20

int realSenseAttached(){
	int frameCount=0;
	//int counter=0;
    try {
        RealSense realsense;
        realsense.startPipeline();
		float depthScale = realsense.depthScale;

		std::vector<cv::Mat> depths(AVG_FRAMES);
    	std::vector<cv::Mat> colorImages(AVG_FRAMES);
        realsense.captureFrames([&depths, &colorImages, &frameCount, &depthScale](const rs2::frameset& frames) 
		{
			
		    //get the average of 5 frames per second
			rs2::depth_frame depth = frames.get_depth_frame();

            rs2::video_frame rgb_frame = frames.get_color_frame();

            rs2::frame depth_processed = preprocessDepth(depth);

            // Convert depth frame to CV_16U Mat
            cv::Mat depth_mat(cv::Size(640, 480), CV_16UC1, (void*)depth_processed.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);

			// Define the row and column range
			//cv::Range rowRange(0, 480); // Rows 1 to 3
			//cv::Range colRange(0, 60); // Columns 2 to 4
			//depth_mat(rowRange, colRange) = cv::Scalar(0);

			colorImages[frameCount] = color_mat.clone();
        	depths[frameCount] = depth_mat.clone();
			frameCount++;

			if(frameCount == AVG_FRAMES)
        	{
				PlaneDetection plane_detection;

				cv::Mat avgColor = computeAverage(colorImages);
				cv::Mat avgDepth = computeAverage(depths);
				
				plane_detection.readDepthImage(avgDepth);
				plane_detection.readColorImage(avgColor);


				plane_detection.runPlaneDetection();

				Surfaces surfaces(plane_detection);

				plane_detection.plane_filter.colors = surfaces.colors;
				plane_detection.plane_filter.publicRefineDetails(&plane_detection.plane_vertices_, nullptr, &plane_detection.seg_img_);
				plane_detection.plane_filter.colors = {};

				cv::Mat floorHeat;
				floorHeat = avgColor.clone();

				cv::Mat groundVecs;
				groundVecs = avgColor.clone();

				float max_distance = 0;

				if (surfaces.groundIdx!=-1){
					Plane plane = computePlaneEq(surfaces.planes, surfaces.groundIdx);
					std::vector<Eigen::Vector3d> projectedVertices = projectOnPlane(surfaces.vertices, plane);

					pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud = makeVoxelCloud(projectedVertices, plane_detection.plane_vertices_);

					floorHeat = drawFloorHeatMap(surfaces.vertices, 
						plane_detection.plane_vertices_, surfaces.groundIdx, voxelCloud, plane, avgColor);

					std::tie(groundVecs, max_distance) = drawDistanceVectors(avgColor, plane_detection, surfaces);
				}

				/*std::string formattedString = std::to_string(counter);
				std::string fileName = ".samples/image" + formattedString + ".png";
				cv::imwrite(fileName, floorHeat);
				counter++;*/

				cv::imshow("Processed Frame", floorHeat);
				cv::imshow("Processed Frame 2",  groundVecs);
				cv::imshow("Processed Frame 3",  plane_detection.seg_img_);
				if (cv::waitKey(1) == 27) { // Exit on ESC key
					//cv::imwrite("sample_segmentation.png", plane_detection.seg_img_);
					//cv::imwrite("depth_image.png", depth_mat*50);
					//cv::imwrite("raw_image.png", color_mat);
					std::exit(0);
				}
				frameCount=0;
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

int loadProcessDefault(){
	PlaneDetection plane_detection;

	std::string colorImagePath = "data/raw_image.png";
    std::string depthImagePath = "data/depth_image.png";

	cv::Mat colorImage = cv::imread(colorImagePath, cv::IMREAD_COLOR);
	cv::Mat depthImage = cv::imread(depthImagePath, cv::IMREAD_UNCHANGED);

	plane_detection.readDepthImage(depthImage);
	plane_detection.readColorImage(colorImage);

	// removed the plotting and moved to public member so we can change
	// the colors and plot according to the logic out here
	plane_detection.runPlaneDetection();

	Surfaces surfaces(plane_detection);

	plane_detection.plane_filter.colors = surfaces.colors;
	plane_detection.plane_filter.publicRefineDetails(&plane_detection.plane_vertices_, nullptr, &plane_detection.seg_img_);
	plane_detection.plane_filter.colors = {};

	Plane plane = computePlaneEq(surfaces.planes, surfaces.groundIdx);
	std::vector<Eigen::Vector3d> projectedVertices = projectOnPlane(surfaces.vertices, plane);

	pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud = makeVoxelCloud(projectedVertices, plane_detection.plane_vertices_);

	Forces forces = resultantForces(voxelCloud);

	cv::Mat floorHeat = drawFloorHeatMap(surfaces.vertices, 
		plane_detection.plane_vertices_, surfaces.groundIdx, voxelCloud, plane, colorImage);

	// Display the image in a loop
    while (true) {
        cv::imshow("Processed Frame", floorHeat);
		cv::imshow("Processed Frame 2",  depthImage*50);
		cv::imshow("Processed Frame 3",  plane_detection.seg_img_);
			
        // Wait for 1 ms and check if the ESC key is pressed
        int key = cv::waitKey(1);
        if (key == 27) { // ESC key has ASCII code 27
            break;
        }
    }

    // Clean up and close the window
    cv::destroyAllWindows();
	return EXIT_SUCCESS;
}


int main(int argc, char* argv[]) {
 	if (argc < 2) {
        int response = realSenseAttached();
        return response;
    } else {
        std::string arg2 = argv[1]; // Convert the C-style string to a std::string

        if (arg2 == "-nocam") {
            int response = loadProcessDefault();
            return response;
        } else {
            std::cerr << "Usage: " << argv[0] << " <-nocam>, or execute without arguments" << std::endl;
			return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}