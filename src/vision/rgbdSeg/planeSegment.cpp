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


void fillHoles(const cv::Mat& src) {
	int kernelSize = 16;

    // Split the source image into its color channels
    std::vector<cv::Mat> channels;
    cv::split(src, channels);

    // Define the structuring element
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));

    // Process each channel separately
    for (auto& channel : channels) {
        // Ensure the channel is binary
        cv::Mat binary;
        if (channel.type() != CV_8U) {
            channel.convertTo(binary, CV_8U);
        } else {
            binary = channel;
        }

        // Threshold the image to ensure it's binary
        cv::threshold(binary, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // Apply the closing operation
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

        // Replace the channel with the processed binary image
        channel = binary;
    }

    // Merge the processed channels back together
    cv::merge(channels, src);
}


// Function to apply dilation and erosion on a mask
void processMask(cv::Mat& mask, int dilationSize = 5, int erosionSize = 5) {
    // Create structuring elements for dilation and erosion
    cv::Mat elementDilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilationSize, dilationSize));
    cv::Mat elementErode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erosionSize, erosionSize));

    // Apply dilation followed by erosion
    cv::dilate(mask, mask, elementDilate);
    cv::erode(mask, mask, elementErode);

	// helps to get continuous vector
	fillHoles(mask);
}

cv::Mat drawDistanceVectors(cv::Mat image, PlaneDetection& plane_detection, Surfaces& surfaces){
    /*NOTE: We are using the image with the masked colors that has been dilated and eroded, because
    there are some random dots in the original mask which mess with the distance calculation. This
    is a hacky way of doing this, but its simple.
    */
   	int stride = 32;

	cv::Mat drawnImage = image.clone(); 
    processMask(drawnImage);

    // Define arrow properties
    cv::Scalar color(255, 0, 0); // Blue color
    int thickness = 2; // Thickness of the line and arrowhead
    int lineType = cv::LINE_AA; // Line type (antialiased)
    int shift = 0; // Number of fractional bits in the point coordinates
    double tipLength = 0.1; // Length of the arrow tip in relation to the arrow length

    if (surfaces.groundIdx > -1){
        int col = plane_detection.cloud.width() / 2 - 1;
		for (int maxJ = col-4*stride; maxJ<=col+4*stride; maxJ+=stride){
			bool onSurface = false;
			double maxZ = 0;
			int maxI = -1;
			for (int i = plane_detection.cloud.height()-1; i >= 0; i--){
				// Access the pixel value
				cv::Vec3b pixelValue = image.at<cv::Vec3b>(i, maxJ);

				// this is the correct way of checking for membership of the vertex/pixel in the groundVertices,
				// but we are using the mask instead because its easier to clean up the mask than it is the cloud 
				//std::vector<int> groundVertices = plane_detection.plane_vertices_[surfaces.groundIdx];
				//int pixelIdx = i * plane_detection.cloud.width() + col;
				//auto inclusion = std::find(groundVertices.begin(), groundVertices.end(), pixelIdx);
				//if (inclusion != groundVertices.end()) {

				// Check if the pixel value is approximately green
				if (std::abs(pixelValue[1] - 255) < 20 && pixelValue[0] < 20 && pixelValue[2] < 20) {
					if (!onSurface){
						onSurface=true;
					}
					double x, y, z;
					plane_detection.cloud.get(i, maxJ, x, y, z);

					if (z > maxZ){
						maxZ = z;
						maxI = i;
					}

				// if we are on surface and have reached a pixel of another class, break early
				}else if (onSurface){
					break;
				}
			}

			// Ensure maxI was updated
			if (maxI != -1) {
				// Define start and end points for the arrow
				cv::Point start(maxJ,  plane_detection.cloud.height()-1);
				cv::Point end(maxJ, maxI);

				// Draw the arrowed line
				cv::arrowedLine(drawnImage, start, end, color, thickness, lineType, shift, tipLength);
			}

			}

    }
	return drawnImage;
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

			cv::Mat drawnImage = drawDistanceVectors(plane_detection.seg_img_, plane_detection, surfaces);

			//std::cout << plane_detection.cloud.vertices.size() << std::endl;
			//std::cout << total << std::endl;

            //plane_detection.prepareForMRF();
            //runMRFOptimization();

            // Display the frame
            //cv::imshow("Processed Frame", color_mat);

			cv::imshow("Processed Frame", drawnImage);
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