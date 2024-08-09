#include "plane_detection.h"
#include <Eigen/Eigen>
#include "artificialFields.hpp"

#define AGENT_WIDTH 25 // cm

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

	std::vector<double> _vecMaxZ; // the real max Z, considering the agent width
	std::vector<int> _vecMaxI; // the real max I 

    GroundVectors(int vecCount, int stride, double agentWidth = AGENT_WIDTH)
        : vecCount(vecCount), stride(stride), agentWidth(agentWidth) {
            vecMaxJ.resize(vecCount, 0); // Initialize vecMaxZ with vecCount elements set to 0.0
            vecMaxI.resize(vecCount, 0); // Initialize vecMaxI with vecCount elements set to 0
			_vecMaxZ.resize(vecCount, 0.0);
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
            int tolerance = 10;
			for (i = cloud.height()-1; i >= iMax; i--){
                int hitObject = false;

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

                    // if we are outside the width of the agent, continue
					if (abs(x-xR)>agentWidth){
						break;
					}
					cv::Vec3b pixelValue = mask.at<cv::Vec3b>(i, jR);
					if (!(std::abs(pixelValue[1] - 255) < 20 && pixelValue[0] < 20 && pixelValue[2] < 20)) {
						hitObject=true;
						break;
					}
				}

                if (hitObject){
					tolerance--;
                    if (tolerance==0){
                        break;
                    }else{
                        continue;
                    }
				}

				// next check the left
				int jL = j;
				while (true){
					jL--;
					if (jL<0){
						break;
					}
					bool exists = cloud.get(i, jL, xL, y, z);

                    if (!exists){
					    continue;
				    }

                    // if we are outside the width of the agent, continue
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
					tolerance--;
				}
                if (tolerance==0){
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

std::tuple<cv::Mat, double> drawDistanceVectors(cv::Mat image, PlaneDetection& plane_detection, Surfaces& surfaces, cv::Mat mask){
    /*NOTE: We are using the image with the masked colors that has been dilated and eroded, because
    there are some random dots in the original mask which mess with the distance calculation. This
    is a hacky way of doing this, but its simple.
    */
   cv::Mat drawnImage = image.clone(); 

   double max_distance = 0.0;

   	if (surfaces.groundIdx==-1){
		return std::make_tuple(drawnImage, max_distance);
	}

    processMask(mask);

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
			cv::Vec3b pixelValue = mask.at<cv::Vec3b>(i, maxJ);
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
	ground_vecs.finalize(mask);

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

    // is this isn't a null pointer
    max_distance = ground_vecs._vecMaxZ[nVecs/2];

	return std::make_tuple(drawnImage, max_distance);
}

cv::Mat makeMask(PlaneDetection& plane_detection, int groundIdx){
    int rows = plane_detection.cloud.height();
    int cols = plane_detection.cloud.width();

    cv::Mat mask(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for (size_t i = 0; i < plane_detection.plane_vertices_.size(); ++i) {
        for (size_t j = 0; j < plane_detection.plane_vertices_[i].size(); ++j) {
            int idx = plane_detection.plane_vertices_[i][j];
            int row = idx / cols;
            int col = idx % cols;

            int r, g;
            if (i==groundIdx){
                r=0;
                g=255;
            }else{
                r=255;
                g=0;
            }

            cv::Vec3b color(static_cast<uchar>(0),  // Blue channel
                            static_cast<uchar>(g),  // Green channel
                            static_cast<uchar>(r));  // Red channel

            mask.at<cv::Vec3b>(row, col) = color;
        }
    }

    return mask;
}