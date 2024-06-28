#include "plane_detection.h"
#include "realsense.hpp"


int findMinZVal(const std::vector<std::vector<int>>& plane_vertices, std::vector<VertexType>& vertices, std::vector<bool> candidates) {
    int minIndex = -1;
    int minValue = std::numeric_limits<int>::max(); // Start with a large value

    for (size_t i = 0; i < plane_vertices.size(); ++i) {
		if (candidates[i]){
			const std::vector<int>& innerVec = plane_vertices[i];
			for (size_t j = 0; j < innerVec.size(); ++j){
				double secondElementValue = vertices[innerVec[j]].x();
				if (secondElementValue==0){
					continue;
				}
				if (secondElementValue < minValue) {
					minValue = secondElementValue;
					minIndex = static_cast<int>(i); // Record the index of the minimum
				}
			}
		}
    }
    return minIndex;
}

struct Surfaces{
	std::vector<ahc::PlaneSeg::shared_ptr> planes;
	std::vector<vector<int>> plane_vertices;
	std::vector<VertexType> vertices;
	std::vector<int> angles;

	size_t numPlanes;
	int groundIdx; // index of the ground, if it can't be assigned, gives warning

	Surfaces(const PlaneDetection& planeDetection) {
        
		// angles are heading directions, we are going to use these
		// to calculate how far we can go in each direction. 
		for (int angle = -45; angle <= 45; angle += 5) {
            angles.push_back(angle);
        }
		
		planes = planeDetection.plane_filter.extractedPlanes;
		plane_vertices = planeDetection.plane_vertices_;
		vertices = planeDetection.cloud.vertices;
		numPlanes = planeDetection.plane_num_;
		groundIdx = assignGround();
    }

	int assignGround(){
		/* looking for the vector with y-component closest to -1. if there are multiple 
		within tolernace, choose whichever is closest to the bottom (has the smallest y-component).
		*/
		double tolerance = 0.1;
		int nCandidates = 0;
		std::vector<bool> candidates(numPlanes);
		for (int i = 0; i < numPlanes; ++i)
		{
			double normal = planes[i]->normal[1]; // second element is y
			std::cout << "plane number: " << i << "y-val: " << normal << std::endl;  
			if (std::abs(-1.f -  normal) < tolerance){
				candidates[i] = true;
				nCandidates++;
			}else{
				candidates[i] = false;
			} ;
		}

		if (nCandidates==0){
			return -1;
		}else if (nCandidates==1){
			for (size_t i = 0; i < candidates.size(); ++i) {
				if (candidates[i]) {
					return i;
				}
    		}
		}else{
			int minVal = findMinZVal(plane_vertices, vertices, candidates);
			return minVal;
		}
		return 0;
	}

};

int main() {

	PlaneDetection plane_detection;

    try {
        RealSense realsense;
        realsense.startPipeline();

        realsense.captureFrames([&plane_detection](const rs2::frameset& frames) {
            rs2::depth_frame depth = frames.get_depth_frame();
            rs2::video_frame rgb_frame = frames.get_color_frame();

            rs2::frame depth_processed = preprocessDepth(depth);

            // Convert depth frame to CV_16U Mat
            cv::Mat depth_mat(cv::Size(640, 480), CV_16UC1, (void*)depth_processed.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);

            plane_detection.readDepthImage(depth_mat);
            plane_detection.readColorImage(color_mat);
            plane_detection.runPlaneDetection();

			Surfaces surfaces(plane_detection);

            //plane_detection.prepareForMRF();
            //runMRFOptimization();

            // Display the frame
            cv::imshow("Processed Frame", plane_detection.seg_img_);
            if (cv::waitKey(1) == 27) { // Exit on ESC key
				cv::imwrite("sample_segmentation.png", plane_detection.seg_img_);
				cv::imwrite("depth_image.png", depth_mat);
				cv::imwrite("raw_image.png", color_mat);
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