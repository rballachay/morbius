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
				plane_detection.plane_filter.publicRefineDetails(&plane_detection.plane_vertices_, nullptr, &plane_detection.seg_img_);

				Surfaces surfaces(plane_detection);

				cv::Mat floorHeat = avgColor.clone();
				cv::Mat groundVecs = avgColor.clone();
				float max_distance = 0;

				cv::Mat mask = plane_detection.seg_img_;

				if (surfaces.groundIdx!=-1){
					mask = makeMask(plane_detection, surfaces.groundIdx);

					Plane plane = computePlaneEq(surfaces.planes, surfaces.groundIdx);
					std::vector<Eigen::Vector3d> projectedVertices = projectOnPlane(surfaces.vertices, plane);

					pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud = makeVoxelCloud(projectedVertices, plane_detection.plane_vertices_);

					floorHeat = drawFloorHeatMap(surfaces.vertices, 
						plane_detection.plane_vertices_, surfaces.groundIdx, voxelCloud, plane, avgColor);

					std::tie(groundVecs, max_distance) = drawDistanceVectors(avgColor, plane_detection, surfaces, mask);
				}

				cv::imshow("Processed Frame", floorHeat);
				cv::imshow("Processed Frame 2",  groundVecs);
				cv::imshow("Processed Frame 3",  mask);
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

int main(int argc, char* argv[]) {
	int response = realSenseAttached();
	return response;
}