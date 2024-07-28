#include "realsense.hpp"
#include "plane_detection.h"
#include "artificialFields.hpp"
#include <Eigen/Eigen>
// can be changed to an arg later
#define AVG_FRAMES 10
#define AGENT_WIDTH 20 // cm

struct GroundVectors{
	/* Container for holding the ground vectors and their max
	distances so that we can calculate the max traversible distance
	for each vector on the ground.
	*/
	ImagePointCloud cloud;
	std::vector<int> vecMaxJ; // the initial max J value at each vector
	std::vector<int> vecMaxI; // the initial max I row for each vector
	int vecCount; 
	int stride; 
	double agentWidth; 

	std::vector<int> _vecMaxZ; // the real max Z, considering the agent width
	std::vector<int> _vecMaxI; // the real max I 

    GroundVectors(int vecCount, int stride, double agentWidth = AGENT_WIDTH)
        : vecCount(vecCount), stride(stride), agentWidth(agentWidth) {
            vecMaxJ.resize(vecCount, 0); // Initialize vecMaxZ with vecCount elements set to 0.0
            vecMaxI.resize(vecCount, 0); // Initialize vecMaxI with vecCount elements set to 0
			_vecMaxZ.resize(vecCount, 0);
			_vecMaxI.resize(vecCount, -1);
        }

	void finalize(cv::Mat mask){
		// iterate over all vectors and see how far there is space
		int halfW = agentWidth;

		// check the width 
		for (int k=0; k<vecCount; k++){
			int iMax = vecMaxI[k];
			int j = vecMaxJ[k];
			
			if (iMax==-1){
				continue;
			}

			int i;
			int hitObject = false;
			for (i = cloud.height()-1; i >= iMax; i--){
				double x, y, z, xL, xR;
				
				bool exists = cloud.get(i, j, x, y, z);

				if (!exists){
					continue;
				}

				// first check the right
				int jR = j;
				while (true){
					jR++;
					if (jR>cloud.width()-1){
						break;
					}
					bool exists = cloud.get(i, jR, xR, y, z);

					if (!exists){
						continue;
					}

					if (abs(x-xR)>agentWidth){
						break;
					}
					cv::Vec3b pixelValue = mask.at<cv::Vec3b>(i, jR);
					if (!(std::abs(pixelValue[1] - 255) < 20 && pixelValue[0] < 20 && pixelValue[2] < 20)) {
						hitObject=true;
						break;
					}
				}

				// next check the left
				int jL = j;
				while (true){
					jL--;
					if (jL<0){
						break;
					}
					cloud.get(i, jL, xL, y, z);
					if (abs(x-xL)>agentWidth){
						break;
					}
					cv::Vec3b pixelValue = mask.at<cv::Vec3b>(i, jL);
					if (!(std::abs(pixelValue[1] - 255) < 20 && pixelValue[0] < 20 && pixelValue[2] < 20)) {
						hitObject=true;
						break;
					}
				}

				if (hitObject){
					break;
				}
					
			}

			double x,y,z;
			cloud.get(i, j, x, y, z);
			_vecMaxZ[k] = z;
			_vecMaxI[k] = i;

		}
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
	int kernelSize = 15;

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
   cv::Mat drawnImage = image.clone(); 

   	if (surfaces.groundIdx==-1){
		return drawnImage;
	}

    processMask(drawnImage);

    // Define arrow properties
    cv::Scalar color(255, 0, 0); // Blue color
    int thickness = 2; // Thickness of the line and arrowhead
    int lineType = cv::LINE_AA; // Line type (antialiased)
    int shift = 0; // Number of fractional bits in the point coordinates
    double tipLength = 0.1; // Length of the arrow tip in relation to the arrow length

	int stride = 32;
	int nVecs = 7 ; // has to be odd
	GroundVectors ground_vecs(nVecs, stride, AGENT_WIDTH);
	ground_vecs.cloud = plane_detection.cloud;

	// Create a vector to store maxJ values
	int col = plane_detection.cloud.width() / 2 - 1;
    std::vector<int> maxJValues;
    for (int j = col-(nVecs/2)*stride; j <=col+(nVecs/2)*stride; j += stride) {
        maxJValues.push_back(j);
    }

	for (int k; k < maxJValues.size(); k++){
		int maxJ = maxJValues[k];
		ground_vecs.vecMaxJ[k] = maxJ; 
		bool onSurface = false;
		double maxZ = 0;
		int maxI = -1;
		for (int i = plane_detection.cloud.height()-1; i >= 0; i--){
			// this is the correct way of checking for membership of the vertex/pixel in the groundVertices,
			// but we are using the mask instead because its easier to clean up the mask than it is the cloud 
			//std::vector<int> groundVertices = plane_detection.plane_vertices_[surfaces.groundIdx];
			//int pixelIdx = i * plane_detection.cloud.width() + col;
			//auto inclusion = std::find(groundVertices.begin(), groundVertices.end(), pixelIdx);
			//if (inclusion != groundVertices.end()) {

			// Check if the pixel value is approximately green
			cv::Vec3b pixelValue = image.at<cv::Vec3b>(i, maxJ);
			if (std::abs(pixelValue[1] - 255) < 20 && pixelValue[0] < 20 && pixelValue[2] < 20) {
				if (!onSurface){
					onSurface=true;
				}
				double x, y, z;
				plane_detection.cloud.get(i, maxJ, x, y, z);

				if (z > maxZ){
					maxZ = z;
					maxI = i;
					ground_vecs.vecMaxI[k] = i; 
				}

			// if we are on surface and have reached a pixel of another class, break early
			}else if (onSurface){
				break;
			}
		}


	}
	ground_vecs.finalize(drawnImage);

	for (int k; k < maxJValues.size(); k++){
		
		// if the final I is the end, or -1, skip drawing the arrow
		if (ground_vecs._vecMaxI[k] == -1)
		{
			continue;
		}
		
		// Define start and end points for the arrow
		cv::Point start(ground_vecs.vecMaxJ[k],  plane_detection.cloud.height()-1);
		cv::Point end(ground_vecs.vecMaxJ[k], ground_vecs._vecMaxI[k]);
		cv::arrowedLine(drawnImage, start, end, color, thickness, lineType, shift, tipLength);


		// Define start and end points for the arrow
		//cv::Point start(ground_vecs.vecMaxJ[k],  plane_detection.cloud.height()-1);
		//cv::Point end(ground_vecs.vecMaxJ[k], ground_vecs.vecMaxI[k]);

		// Draw the arrowed line
		//cv::arrowedLine(drawnImage, start, end, color, thickness, lineType, shift, tipLength);
	}

	return drawnImage;
}

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

				plane_detection.plane_filter.publicRefineDetails(&plane_detection.plane_vertices_, nullptr, &plane_detection.seg_img_);
				
				Surfaces surfaces(plane_detection);

				cv::Mat floorHeat;
				floorHeat = avgColor.clone();

				if (surfaces.groundIdx!=-1){
					Plane plane = computePlaneEq(surfaces.planes, surfaces.groundIdx);
					std::vector<Eigen::Vector3d> projectedVertices = projectOnPlane(surfaces.vertices, plane);

					pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud = makeVoxelCloud(projectedVertices, plane_detection.plane_vertices_);

					floorHeat = drawFloorHeatMap(surfaces.vertices, 
						plane_detection.plane_vertices_, surfaces.groundIdx, voxelCloud, plane, avgColor);
				}

				/*std::string formattedString = std::to_string(counter);
				std::string fileName = ".samples/image" + formattedString + ".png";
				cv::imwrite(fileName, floorHeat);
				counter++;*/

				cv::imshow("Processed Frame", floorHeat);
				cv::imshow("Processed Frame 2",  depth_mat*50);
				//cv::imshow("Processed Frame 3",  plane_detection.seg_img_);
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