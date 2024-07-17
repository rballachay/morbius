/** 
#include <opencv2/opencv.hpp>
#include <stdint.h>

cv::Mat computeAverage(const std::vector<cv::Mat>& mats) {
    
   // <[640,480,3],[640,480,3]> -> [640,480,3]
    
    if (mats.empty()) {
        std::cerr << "Error: Input vector is empty!" << std::endl;
        return cv::Mat();
    }

    // Initialize sum for each channel
    std::vector<cv::Mat> channelSums;
    for (int c = 0; c < mats[0].channels(); ++c) {
        cv::Mat channelSum = cv::Mat::zeros(mats[0].size(), CV_64FC1); // Sum in double for accuracy
        channelSums.push_back(channelSum);
    }

    // Accumulate sums across all images
    for (const auto& mat : mats) {
        std::vector<cv::Mat> channels;
        cv::split(mat, channels); // Split into separate channels

        for (int c = 0; c < mat.channels(); ++c) {
            cv::Mat channelFloat;
            channels[c].convertTo(channelFloat, CV_64FC1); // Convert to double for accumulation
            channelSums[c] += channelFloat;
        }
    }

    // Compute average for each channel
    std::vector<cv::Mat> channelAverages;
    for (int c = 0; c < mats[0].channels(); ++c) {
        cv::Mat channelAverage;
        cv::divide(channelSums[c], static_cast<double>(mats.size()), channelAverage);
        channelAverages.push_back(channelAverage);
    }

    // Merge channels into single output image
    cv::Mat averageImage;
    cv::merge(channelAverages, averageImage);

    // Convert back to original type (assuming input mats are of same type)
    averageImage.convertTo(averageImage, mats[0].type());

    return averageImage;
}



// g++ test.cpp -o sample  -fpermissive  -std=c++17 `pkg-config --cflags --libs opencv4`

int main(){
    int i=0;
    std::vector<cv::Mat> depths(5);
    std::vector<cv::Mat> colorImages(5);
    while (true){
        std::string colorImagePath = "data/raw_image.png";
        std::string depthImagePath = "data/depth_image.png";

        cv::Mat colorImage = cv::imread(colorImagePath, cv::IMREAD_COLOR);
        cv::Mat depthImage = cv::imread(depthImagePath, cv::IMREAD_UNCHANGED);

        if (colorImage.empty() || depthImage.empty()) {
            std::cerr << "Failed to load image " << colorImagePath << " or " << depthImagePath << std::endl;
            break;
        }

        // write append to depths+colorImages based on i 
        colorImages[i] = colorImage;
        depths[i] = depthImage;

        if(i == 4)
        {
            cv::Mat avgColor = computeAverage(colorImages);
            cv::Mat avgDepth = computeAverage(depths);
            cv::imshow("Processed Frame", avgDepth*20);
            cv::waitKey(0); // Wait for a key press
            break;
        }
        i++;
    }
    
    return 1;
}
*/