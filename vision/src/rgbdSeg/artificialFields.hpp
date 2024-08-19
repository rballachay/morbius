#ifndef ARTIFICIAL_FIELDS_H
#define ARTIFICIAL_FIELDS_H

#include <memory>
#include <cmath>
#include "plane_detection.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#define C_ATTRACT 5.f // coefficient of attraction of destination point
#define C_REPULSE 0.5f // coefficient of repulsion of each point in voxel cloud
#define MAX_DIST 250.f // max distance to apply force in artificial field
#define VOXEL_DENSITY 10.f // the distance we want between each voxel, in mm
#define STEP_SIZE 20.f // step size in mm in the walking algorithm
#define DIST_THRESH 100.f // tolerance in cm for 'arrival' at destiation

struct Surfaces{
    /*
    Container that holds the 'surfaces' - the result of running PlaneSegment
    algorithm. helps to determine which of the planes is the ground. 

    The 'ground' is whatever plane as a normal vector closest to negative 1,
    aka whatever surface is facing the most 'up' 
    */
	std::vector<ahc::PlaneSeg::shared_ptr> planes; // vector of plane objects - contains normals
	std::vector<std::vector<int>> plane_vertices; // indices of vertices that belong to plane i
	std::vector<Eigen::Vector3d> vertices; // all the vertices as 3vec
	std::vector<cv::Vec3b> colors; // colors, to be plotted using seg.img_

	size_t numPlanes;
	int groundIdx = -1; // index of the ground, if it can't be assigned, gives warning

	Surfaces(PlaneDetection planeDetection) {
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
		std::vector<bool> candidates(numPlanes);
		int minIndex = -1;
		double minDiff = 1.;
		for (int i = 0; i < numPlanes; i++)
		{
			if (!planes[i]) continue; // if nullptr, skip 
			double normal = planes[i]->normal[1]; // second element is y value
			std::cout << "plane number: " << i << ", y-val: " << normal << std::endl;  
			double diff = std::abs(-1.f -  normal);
			if (diff < minDiff){
				minDiff=diff;
				minIndex=i;
			}
		}
        std::cout << "Final ground index: " << minIndex << std::endl;
		return minIndex;
	}

};

struct Forces{
    double x,z;
};

struct Plane {
    double a, b, c, d;

    Plane(double _a, double _b, double _c, double _d)
        : a(_a), b(_b), c(_c), d(_d) {
    }
};

cv::Mat computeAverage(const std::vector<cv::Mat>& mats) {
    /* Compute the average of a vector of matrices, assuming 
    all have the same size. Used as multiple depth frames are collected
    and averaged prior to running plane segmentation.
    */
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

Plane computePlaneEq(std::vector<ahc::PlaneSeg::shared_ptr> planeSegs, int groundIdx) {
    /* Calculate the equation of the 'ground plane' based on the center point 
    and normal vector so that we can project our points onto the ground. Note that 
    we assume that everything in the room is a 'cylinder', so even if the object is hanging 
    at a height high enough the robot, it will occupy a cylinder/rectangle that reaches
    all the way to the ground.
    */
    Plane plane(0,0,0,0);
    if (groundIdx==-1){
        return plane;
    }

    ahc::PlaneSeg::shared_ptr planeSeg = planeSegs[groundIdx];
    plane.a = planeSeg->normal[0];
    plane.b = planeSeg->normal[1];
    plane.c = planeSeg->normal[2];
    plane.d = -(plane.a * planeSeg->center[0] + plane.b * planeSeg->center[1] + plane.c * planeSeg->center[2]);
    return plane;
}

std::vector<VertexType> projectOnPlane(std::vector<VertexType> vertices, Plane plane) {
    /* Project all the vertices in the point cloud onto the ground so that we can 
    compute our artificial potential field. Note that we are going to filter the cloud after, 
    so that many points that belong to one object aren't counted multiple times. 
    */
    std::vector<VertexType> projectedPoints;

    for (size_t i = 0; i < vertices.size(); ++i) {
        VertexType point = vertices[i];
        VertexType projectedPoint;
        double t = -(plane.a * point.x() + plane.b * point.y() + plane.c * point.z() + plane.d) / (plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);
        projectedPoint.x() = point.x() + plane.a * t;
        projectedPoint.y() = point.y() + plane.b * t;
        projectedPoint.z() = point.z() + plane.c * t;
        projectedPoints.push_back(projectedPoint);
    }
    return projectedPoints;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr inputCloud,
                                                      double min_x = -1000, double max_x = 1000,
                                                      double min_y = -1000, double max_y = 1000,
                                                      double min_z = 0, double max_z = 2000) {

    /* Two step filtering of the point cloud
    1. Filter the point cloud to only keep points within a certain range of values.
    2. Reduce the density of points in the point cloud to 1 every 10 mm
    */                                                    
    pcl::PointCloud<pcl::PointXYZL>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZL>);

    // stage 1 of filtering - 
    pcl::PassThrough<pcl::PointXYZL> pass;
    pass.setInputCloud(inputCloud);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*filteredCloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*filteredCloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*filteredCloud);

    // stage 2 of filtering - reduce point density
    pcl::PointCloud<pcl::PointXYZL>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::VoxelGrid<pcl::PointXYZL> sor;
    sor.setInputCloud(filteredCloud);
    sor.setLeafSize(VOXEL_DENSITY, VOXEL_DENSITY, VOXEL_DENSITY); // the units here are mm -> 0.01 m
    sor.filter(*finalCloud);

    return finalCloud;
}


pcl::PointCloud<pcl::PointXYZL>::Ptr makeVoxelCloud(const std::vector<VertexType>& vertices, 
        const std::vector<std::vector<int>>& plane_vertices) {
    /*
    */
    // Step 2: Create a pcl::PointCloud<pcl::PointXYZ> and populate it
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZL>);
    for (int i=0; i<plane_vertices.size(); i++){
        for (int j=0; j<plane_vertices[i].size(); j++) {
            int idx = plane_vertices[i][j];
            VertexType vertex = vertices[idx];
            pcl::PointXYZL point;
            point.x = vertex.x();
            point.y = vertex.y();
            point.z = vertex.z();
            point.label = i;
            cloudXYZ->points.push_back(point);
        }
    }

    cloudXYZ->width = cloudXYZ->points.size();
    cloudXYZ->height = 1; // Unorganized point cloud
    cloudXYZ->is_dense = true;

    std::cerr << "PointCloud before filtering: " << cloudXYZ->width * cloudXYZ->height 
              << " data points." << std::endl;

    // filter out max values and reduce density
    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud = filterPointCloud(cloudXYZ);

    std::cerr << "PointCloud after filtering: " << filtered_cloud->width * filtered_cloud->height 
              << " data points." << std::endl;

    return filtered_cloud;
}

double calcModulus(double a, double b) {
    return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
}

Forces resultantForces(pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud, 
        pcl::PointXYZ source = pcl::PointXYZ(0,0,0), pcl::PointXYZ dest = pcl::PointXYZ(0,0,5000),
        const double c_attract = C_ATTRACT, const double c_repulse = C_REPULSE, const double d0 = MAX_DIST){
    /*
    Based on the principle of "artificial potential field" used for local navigation
    in autonomous robots. There are two forces: attaction to the desination, which is a point 
    in local space where the robot is trying to navigate. The second is all the objects in space which
    are not the floor, all of which have a repelling force. Summing these forces, we get a heading 
    vector which determines the trajectory with the lowest resistance. 

    This particular implementation is based on:
    'A 3D Anti-collision System based on Artificial Potential Field Method for a Mobile Robot' 

    for the point-cloud implementation. However this didn't work particularly well when implemented, 
    so the actual distance-to-force calculation, the following paper was used:

    `Artificial Potential Field with Discrete Map Transformation for Feasible Indoor Path Planning`
    */
    int cloudSize = voxelCloud->size();

    double force_x_repulse = std::accumulate(voxelCloud->points.begin(), voxelCloud->points.end(), 0.0,
    [&c_repulse, &d0, &source, &cloudSize](double sum, const pcl::PointXYZL& value) {
        double Ur = 0;
        double dist = calcModulus(value.x - source.x, value.z-source.z);

        // Calculate the vector from source to value
        Eigen::Vector3d source_vec(source.x, 0, source.z);
        Eigen::Vector3d value_vec(value.x, 0, value.z);
        Eigen::Vector3d diff = source_vec - value_vec ;

        // Calculate the angle scale as the cosine of the angle
        double cos_theta = diff.normalized().dot(Eigen::Vector3d(1, 0, 0)); // Assuming angle with respect to the x-axis

        if (dist != 0 && std::abs(dist) <= d0) {
            Ur = 0.5 * c_repulse * std::pow(1.0 / std::abs(dist) - 1.0 / d0, 2);
        }
        return sum + cos_theta * Ur;
    });

    /*double force_z_repulse = std::accumulate(voxelCloud->points.begin(), voxelCloud->points.end(), 0.0,
    [&c_repulse, &d0, &source, &cloudSize](double sum, const pcl::PointXYZL& value) {
        double Ur = 0;
        double dist = calcModulus(value.x - source.x, value.z-source.z);

        // Calculate the vector from source to value
        Eigen::Vector3d source_vec(source.x, 0, source.z);
        Eigen::Vector3d value_vec(value.x, 0, value.z);
        Eigen::Vector3d diff = value_vec - source_vec;

        // Calculate the angle scale as the cosine of the angle
        double cos_theta = diff.normalized().dot(Eigen::Vector3d(0, 0, 1)); // Assuming angle with respect to the z-axis

        if (dist != 0 && std::abs(dist) <= d0) {
            Ur = 0.5 * c_repulse * std::pow(1.0 / std::abs(dist) - 1.0 / d0, 2);
        }
        return sum + cos_theta * Ur;
    });*/

    const double phi = std::atan2(dest.x-source.x, dest.z-source.z);

    double force_z_attr = std::cos(phi)*c_attract/calcModulus(dest.x-source.x, dest.z-source.z);
    double force_x_attr = std::sin(phi)*c_attract/calcModulus(dest.x-source.x, dest.z-source.z);
    Forces forces;

    double force_x = force_x_attr - force_x_repulse;

    // we are ignoring repulsive forces in the z-axis, as we are taking a direct path from 0,0,0 
    // and so it doesn't help to accumulate backwards repulsive forces
    double force_z = force_z_attr; //- force_z_repulse;

    forces.x = force_x / calcModulus(force_x, force_z);
    forces.z = force_z / calcModulus(force_x, force_z);
    return forces;
}

cv::Vec3b valueToColor(double value, double minVal, double maxVal) {
    double normalizedValue = (value - minVal) / (maxVal - minVal);
    int hue = static_cast<int>(120 * (1-normalizedValue));
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    cv::Vec3b color = bgr.at<cv::Vec3b>(0, 0);
    return color;
}

std::vector<cv::Vec3b> valuesToColors(const std::vector<double>& values) {
    /* Map from range of values to blue-to-red, where blue is the minimum
    value and red is the max.
    */

    double minVal = *std::min_element(values.begin(), values.end());
    double maxVal = *std::max_element(values.begin(), values.end());

    std::vector<cv::Vec3b> colors;
    colors.reserve(values.size());

    // Convert each value to BGR color and store in the vector
    for (double value : values) {
        cv::Vec3b color = valueToColor(value, minVal, maxVal);
        colors.push_back(color);
    }

    return colors;
}

pcl::PointXYZ normalizeVector(const pcl::PointXYZ& vec) {
    double magnitude = std::sqrt(vec.x * vec.x  + vec.z * vec.z);
    return pcl::PointXYZ(vec.x / magnitude, 0, vec.z / magnitude);
}

double calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) +  std::pow(p2.z - p1.z, 2));
}

pcl::PointXYZ calculateStep(const pcl::PointXYZ& source, const Forces& dir_forces, double step_size) {
    /* Given the source point and normalized direction vector, calculate the distance we should 
    move in x and z dimensions.
    */
    pcl::PointXYZ direction;
    direction.x = dir_forces.x;
    direction.z = dir_forces.z;

    pcl::PointXYZ normalized_direction = normalizeVector(direction);
    pcl::PointXYZ step(
        source.x + step_size * normalized_direction.x,
        0,
        source.z + step_size * normalized_direction.z
    );
    return step;
}

double shortestDistanceToPath(const pcl::PointXYZ& point, const std::vector<pcl::PointXYZ>& path) {
    /* Calculate shortest distance between point on the ground and collection of 
    points that compose the path of least resistance on the ground.
    */
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& path_point : path) {
        double distance = calculateDistance(point, path_point);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}

cv::Mat drawFloorHeatMap(std::vector<VertexType>& vertices, std::vector<std::vector<int>>& plane_vertices,         
        int groundIdx, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, Plane plane, cv::Mat image, 
        const double step_size = STEP_SIZE, const double dist_thresh = DIST_THRESH, 
        pcl::PointXYZ source = pcl::PointXYZ(0,0,0), pcl::PointXYZ destination = pcl::PointXYZ(0,0,1000)){
    /*
    See readme for full explanation of this algorithm. The general idea is that we have a 2d map of voxels 
    that either belong to the ground or to objects. We have a source point and a destination point on this ground
    and want to walk from the source to the destination using the summary of forces at each step to determine 
    the direction. Each step moves step_size, and we stop once we are within <dist_thresh of our destination.

    Once we have the optimal path, we take our original image and draw the ground, coloring 
    each pixel according to the distance from the optimal path.
    */  
    cv::Mat output_image = image.clone();

    if (groundIdx==-1){
        return output_image;
    }

    std::vector<pcl::PointXYZ> tracedPath;
    tracedPath.push_back(source);

    double previousDistance = calculateDistance(source, destination);

    // continue loop until we reach the end position or run out of tolerance
    int tolerance=10;
    int globalTolerance=50;
    while (true){

        Forces forces = resultantForces(cloud, source, destination);
        
        pcl::PointXYZ new_point = calculateStep(source, forces, step_size);

        source.x=new_point.x;
        source.z=new_point.z;

        tracedPath.push_back(new_point); // add point

        double currentDistance = calculateDistance(source, destination);

        if (currentDistance < dist_thresh) {
            std::cout << "Reached the destination.\n";
            break;
        }
        if (currentDistance >= previousDistance) {
            tolerance--;
            globalTolerance--;
            if (!tolerance){
                std::cout << "No longer approaching the destination. Exiting...\n";
                break;
            }
        }else{
            tolerance=10;
        }

        if (!globalTolerance){
            break;
        }

        previousDistance = currentDistance;

    }

    std::vector<double> distances;
    for (int i : plane_vertices[groundIdx]){
        VertexType vertex = vertices[i];
        pcl::PointXYZ sourceIdx;
        sourceIdx.x = vertex.x();
        sourceIdx.z = vertex.z();

        double dist = shortestDistanceToPath(sourceIdx, tracedPath);
        distances.push_back(dist);
    }

    std::vector<cv::Vec3b> rgbVals = valuesToColors(distances);

    size_t nGround = plane_vertices[groundIdx].size();
    std::vector<std::tuple<int,int>> groundPts(nGround);
    for (size_t i=0; i<nGround; i++){
        size_t idx = plane_vertices[groundIdx][i];

        // Ensure idx is within the valid range of image size
        if (idx >= image.rows * image.cols) {
            std::cerr << "Index out of bounds: " << idx << std::endl;
            continue;
        }

        int row = idx / image.cols;
        int col = idx % image.cols;
        groundPts[i] = std::make_tuple(row, col); 
    }

    for (size_t i=0; i<nGround; i++){
        std::tuple<int,int> tup = groundPts[i];
        int row = std::get<0>(tup);
        int col = std::get<1>(tup);

        // Ensure row and col are within the valid range of the image
        if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) {
            std::cerr << "Pixel position out of bounds: (" << row << ", " << col << ")" << std::endl;
            continue;
        }

        cv::Vec3b originalPixel = image.at<cv::Vec3b>(row, col);
        cv::Vec3b maskPixel = rgbVals[i];

        // Alpha blending
        float alpha = 0.5;
        cv::Vec3b blendedPixel;
        for (int j = 0; j < 3; j++) {
            blendedPixel[j] = static_cast<uchar>(alpha * maskPixel[j] + (1 - alpha) * originalPixel[j]);
        }

        // Ensure row and col are within the valid range of the output image
        if (row < 0 || row >= output_image.rows || col < 0 || col >= output_image.cols) {
            std::cerr << "Output pixel position out of bounds: (" << row << ", " << col << ")" << std::endl;
            continue;
        }

        output_image.at<cv::Vec3b>(row, col) = blendedPixel;
    }


    // Draw x and z values of a few ground points on the image
    /*for (size_t i = 0; i < std::min(nGround, static_cast<size_t>(640*5*10)); i+=640*5){ // Draw for the first 10 points
        std::tuple<int, int> tup = groundPts[i];
        int row = std::get<0>(tup);
        int col = std::get<1>(tup);

        // VertexType vertex = vertices[plane_vertices[groundIdx][i]];
        std::string text = "(" + std::to_string(distances[i]) + ")";
        
        cv::putText(output_image, text, cv::Point(col, row), cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(255, 255, 255), 1);
    }*/

    return output_image;
}

#endif // PLANE_OPERATIONS_H

/*
double scalarProjectionMagnitude(const pcl::PointXYZ& start_point,
                                 const pcl::PointXYZ& end_point,
                                 const pcl::PointXYZ& point_to_project) {
    // Compute vector from start_point to end_point (direction vector)
    pcl::PointXYZ direction_vector;
    direction_vector.x = end_point.x - start_point.x;
    direction_vector.z = end_point.z - start_point.z;

    // Compute dot product of direction_vector and vector from start_point to point_to_project
    double dot_product = (point_to_project.x - start_point.x) * direction_vector.x +
                         (point_to_project.z - start_point.z) * direction_vector.z;

    // Compute magnitude of direction_vector
    double direction_magnitude = std::sqrt(direction_vector.x * direction_vector.x +
                                           direction_vector.z * direction_vector.z);

    // Compute scalar projection magnitude
    double scalar_projection_magnitude = dot_product / direction_magnitude;

    return dot_product;
}


cv::Mat drawFloorHeatMap_old(std::vector<VertexType>& vertices, std::vector<std::vector<int>>& plane_vertices, 
        int groundIdx, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, Plane plane, cv::Mat image){
    
    cv::Mat output_image = image.clone();
    if (groundIdx==-1){
        return output_image;
    }

    size_t nGround = plane_vertices[groundIdx].size();
    std::vector<VertexType> groundVertices(nGround);
    std::vector<std::tuple<int,int>> groundPts(nGround);
    for (size_t i=0; i<nGround;i++){
        size_t idx = plane_vertices[groundIdx][i];
        groundVertices[i] = vertices[idx];

        // from plane_detection.h:44 -> const int pixIdx = row * w + col;
        int row = idx/image.cols;
        int col = idx % image.cols;
        groundPts[i] = std::make_tuple(row, col); 
    }

    std::vector<double> magnitudes(nGround);
    for (size_t i=0; i<nGround;i++){
        VertexType vertex = groundVertices[i];
        pcl::PointXYZ source;
        pcl::PointXYZ destination = pcl::PointXYZ(0,0,1000);
        source.x = vertex.x();
        source.y = vertex.y();
        source.z = vertex.z();
        Forces forces = resultantForces(cloud, source, destination);

        pcl::PointXYZ point;
        point.x = source.x + forces.x*100;
        point.z = source.z + forces.z*100;
        double magnitude = scalarProjectionMagnitude(source, destination, point);
        magnitudes[i] = magnitude;
    } 

    std::vector<cv::Vec3b> rgbVals = valuesToColors(magnitudes);

    for (size_t i=0; i<nGround; i++){
        std::tuple<int,int> tup = groundPts[i];
        int row = std::get<0>(tup);
        double col = std::get<1>(tup);
        output_image.at<cv::Vec3b>(row, col) = rgbVals[i];
    }

    return output_image;
    
}

cv::Mat draw2DPoints(pcl::PointCloud<pcl::PointXYZL>::Ptr points, 
        std::vector<std::vector<int>> planeVertices, int groundIdx, Forces forces){

    // Step 1: Find minimum and maximum x and y values
    int minX = -500;
    int maxX = 500;
    int minZ = 0;
    int maxZ = 1000;

    for (const auto& point : points->points) {
        if (point.x < minX) minX = point.x;
        if (point.x > maxX) maxX = point.x;
        if (point.z < minZ) minZ = point.z;
        if (point.z > maxZ) maxZ = point.z;
    }

    // Step 2: Scale points to range [0, 500]
    for (auto& point : points->points) {
        point.x = static_cast<int>(500.0 * (point.x - minX) / (maxX - minX));
        point.z = static_cast<int>(500.0 * (point.z - minZ) / (maxZ - minZ));
    }

    // Step 3: Create a blank image
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // Step 4: Draw scaled points on the image
    for (size_t i = 0; i < points->points.size(); ++i) {  // Use size_t for the loop variable
        const pcl::PointXYZL& point = points->points[i];
        cv::Scalar color(0, 0, 255); // Default color: Red

        // Change color if point belongs to the ground index
        if (point.label == groundIdx) {
            color = cv::Scalar(0, 255, 0); // Green color for ground index
        }

        // Draw the point on the image
        cv::circle(image, cv::Point(static_cast<int>(point.x),500-static_cast<int>(point.z)), 5, color, -1);
    }

    // Define arrow properties
    cv::Scalar color(0, 0, 255); // Red color
    int thickness = 2;           // Thickness of the arrow
    double tipLength = 0.1;      // Length of the arrow tip
    cv::Point start(250, 500);
    cv::Point end(250-500*forces.x, 500-500*forces.z);

    // Draw the arrow
    cv::arrowedLine(image, start, end, color, thickness, cv::LINE_8, 0, tipLength);

    return image;
}
*/