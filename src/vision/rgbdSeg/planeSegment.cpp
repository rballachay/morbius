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
	std::vector<cv::Vec3b> colors;

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
		numPlanes = planes.size();
		groundIdx = assignGround();

		for (int i=0; i<numPlanes; i++){
			if (i==groundIdx){
				colors.push_back(cv::Vec3b({0,255,0}));
			}else{
				colors.push_back(cv::Vec3b({0,0,255}));
			}
		}
    }

	int assignGround(){
		/* looking for the vector with y-component closest to -1. if there are multiple 
		within tolernace, choose whichever is closest to the bottom (has the smallest y-component).
		*/
		double tolerance = 0.1;
		int nCandidates = 0;
		std::vector<bool> candidates(numPlanes);
		for (int i = 0; i < numPlanes; i++)
		{
			if (!planes[i]) continue; // if nullptr, skip 
			double normal = planes[i]->normal[1]; // second element is y value
			std::cout << "plane number: " << i << ", y-val: " << normal << std::endl;  
			if (std::abs(-1.f -  normal) < tolerance){
				candidates[i] = true;
				nCandidates++;
			}else{
				candidates[i] = false;
			}
		}

		if (nCandidates==0){
			return -1;
		}else if (nCandidates==1){
			for (int i = 0; i < candidates.size(); i++) {
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

void printPlaneMembership(PlaneDetection& plane_detection){
	std::cout << plane_detection.cloud.vertices.size() << std::endl;
	int total = 0;
	for (const auto& innerVec : plane_detection.plane_vertices_) {
		auto maxIter = std::max_element(innerVec.begin(), innerVec.end());
			if (maxIter != innerVec.end()) {
			int maxValue = *maxIter;
			std::cout << "Max value in vector: " << maxValue << std::endl;
		} else {
			std::cout << "Vector is empty" << std::endl;
		}
	}
}

// Function to apply dilation and erosion on a mask
void processMask(cv::Mat& mask, int dilationSize = 5, int erosionSize = 5) {
    // Create structuring elements for dilation and erosion
    cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilationSize, dilationSize));
    cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erosionSize, erosionSize));

    // Apply dilation followed by erosion
    cv::dilate(mask, mask, elementDilate);
    cv::erode(mask, mask, elementErode);
}

void drawDistanceVector(cv::Mat image, PlaneDetection& plane_detection, Surfaces& surfaces){
    /*NOTE: We are using the image with the masked colors that has been dilated and eroded, because
    there are some random dots in the original mask which mess with the distance calculation. This
    is a hacky way of doing this, but its simple.
    */
    processMask(image);

    // Define arrow properties
    cv::Scalar color(255, 0, 0); // Blue color
    int thickness = 2; // Thickness of the line and arrowhead
    int lineType = cv::LINE_AA; // Line type (antialiased)
    int shift = 0; // Number of fractional bits in the point coordinates
    double tipLength = 0.1; // Length of the arrow tip in relation to the arrow length

    if (surfaces.groundIdx > -1){
        int col = plane_detection.cloud.width() / 2;

        std::vector<int> groundVertices = plane_detection.plane_vertices_[surfaces.groundIdx];

        double maxZ = 0;
        int maxI = -1;
        int maxJ = col;
        for (int i = 0; i < plane_detection.cloud.height(); i++){
            // Ensure index is within bounds
            if (i < 0 || i >= image.rows || col < 0 || col >= image.cols) {
                continue;
            }

            // Access the pixel value
            cv::Vec3b pixelValue = image.at<cv::Vec3b>(i, col);

            // Check if the pixel value is approximately green
            if (std::abs(pixelValue[1] - 255) < 20 && pixelValue[0] < 20 && pixelValue[2] < 20) {
                double x, y, z;
                // Ensure cloud.get is safe
                try {
                    plane_detection.cloud.get(i, col, x, y, z);

                    if (z > maxZ){
                        maxZ = z;
                        maxI = i;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error accessing cloud data: " << e.what() << std::endl;
                    continue;
                }
            }
        }

        // Ensure maxI was updated
        if (maxI != -1) {
            // Define start and end points for the arrow
            cv::Point start(maxJ, plane_detection.cloud.height());
            cv::Point end(maxJ, maxI);

            // Draw the arrowed line
            cv::arrowedLine(image, start, end, color, thickness, lineType, shift, tipLength);
        }
    }
}

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

			// removed the plotting and moved to public member so we can change
			// the colors and plot according to the logic out here
            plane_detection.runPlaneDetection();

			Surfaces surfaces(plane_detection);

			plane_detection.plane_filter.colors = surfaces.colors;
			plane_detection.plane_filter.publicRefineDetails(&plane_detection.plane_vertices_, nullptr, &plane_detection.seg_img_);
			plane_detection.plane_filter.colors = {};

			drawDistanceVector(plane_detection.seg_img_, plane_detection, surfaces);

			//std::cout << plane_detection.cloud.vertices.size() << std::endl;
			//std::cout << total << std::endl;

            //plane_detection.prepareForMRF();
            //runMRFOptimization();

            // Display the frame
            //cv::imshow("Processed Frame", color_mat);

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