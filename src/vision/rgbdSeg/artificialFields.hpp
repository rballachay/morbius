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

#define C_ATTRACT 10.
#define C_REPULSE 5.

struct Forces{
    double x,z;
};

// Forward declaration
struct Plane {
    double a, b, c, d;

    Plane(double _a, double _b, double _c, double _d)
        : a(_a), b(_b), c(_c), d(_d) {
    }
};

Plane computePlaneEq(std::vector<ahc::PlaneSeg::shared_ptr> planeSegs, int groundIdx) {
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

double calcAttraction(double x, double y, double x_goal, double y_goal, double C) {
    return C * std::sqrt(std::pow(x - x_goal, 2) + std::pow(y - y_goal, 2));
}

std::vector<VertexType> projectOnPlane(std::vector<VertexType> vertices, Plane plane) {
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

pcl::PointCloud<pcl::PointXYZL>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr inputCloud,
                                                      double min_x = -1000, double max_x = 1000,
                                                      double min_y = -1000, double max_y = 1000,
                                                      double min_z = 0, double max_z = 2000) {
    pcl::PointCloud<pcl::PointXYZL>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZL>);

    // Create the filter object
    pcl::PassThrough<pcl::PointXYZL> pass;
    pass.setInputCloud(inputCloud);

    // Set filter limits
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*filteredCloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*filteredCloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*filteredCloud);

    return filteredCloud;
}


pcl::PointCloud<pcl::PointXYZL>::Ptr makeVoxelCloud(const std::vector<VertexType>& vertices, const std::vector<std::vector<int>>& plane_vertices) {
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

    // filter out max values
    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud = filterPointCloud(cloudXYZ);

    std::cerr << "PointCloud after passthrough filtering: " << filtered_cloud->width * filtered_cloud->height 
              << " data points." << std::endl;

    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::VoxelGrid<pcl::PointXYZL> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setLeafSize(10.f, 10.f, 10.f); // the units here are mm -> 0.01 m
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
              << " data points." << std::endl;

    return cloud_filtered;
}

double computeManhattanDistance(const pcl::PointXYZL& pt1, const pcl::PointXYZ& pt2) {
    return std::abs(pt1.x - pt2.x) + std::abs(pt1.y - pt2.y) + std::abs(pt1.z - pt2.z);
}

double findLargestManhattanDistance(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, const pcl::PointXYZ& referencePoint) {
    double maxDistance = 0.0;

    for (const auto& point : cloud->points) {
        double distance = computeManhattanDistance(point, referencePoint);
        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }

    return maxDistance;
}

double calcModulus(double a, double b) {
    return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
}

Forces resultantForces(pcl::PointCloud<pcl::PointXYZL>::Ptr voxelCloud, 
        pcl::PointXYZ source = pcl::PointXYZ(0,0,0), pcl::PointXYZ dest = pcl::PointXYZ(0,0,5000)){

    double Dmax = findLargestManhattanDistance(voxelCloud, source);
    const double lambda = 0.1;
    const double hat = 500;
    std::vector<double> thetas;
    for (const auto& pt : voxelCloud->points) {
        double arctan_xz = std::atan2(pt.x-source.x, pt.z-source.z);
        thetas.push_back(arctan_xz);
    }

    double force_x_repulse = std::accumulate(thetas.begin(), thetas.end(), 0.0, [&lambda, &Dmax](double sum, double value) {
        return sum + (lambda/Dmax)* std::sin(value);
    });

    double force_z_repulse = std::accumulate(thetas.begin(), thetas.end(), 0.0, [&lambda, &Dmax](double sum, double value) {
        return sum + (lambda/Dmax)* std::cos(value);
    });

    const double phi = std::atan2(dest.x-source.x, dest.z-source.z);

    double force_z_attr = std::cos(phi)*hat/calcModulus(dest.x-source.x, dest.z-source.z);
    double force_x_attr = std::sin(phi)*hat/calcModulus(dest.x-source.x, dest.z-source.z);
    Forces forces;
    double force_x = force_x_attr + force_x_repulse;
    double force_z = force_z_attr - force_z_repulse;

    forces.x = force_x / calcModulus(force_x, force_z);
    forces.z = force_z / calcModulus(force_x, force_z);
    return forces;
}

cv::Vec3b valueToColor(double value, double minVal, double maxVal) {
    // Normalize the value between 0 and 1
    double normalizedValue = (value - minVal) / (maxVal - minVal);

    int hue = static_cast<int>(120 * (1-normalizedValue));
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

    cv::Vec3b color = bgr.at<cv::Vec3b>(0, 0);
    
    return color;
}

std::vector<cv::Vec3b> valuesToColors(const std::vector<double>& values) {
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


cv::Mat drawFloorHeatMap(std::vector<VertexType>& vertices, std::vector<std::vector<int>>& plane_vertices, 
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

pcl::PointXYZ normalizeVector(const pcl::PointXYZ& vec) {
    double magnitude = std::sqrt(vec.x * vec.x  + vec.z * vec.z);
    return pcl::PointXYZ(vec.x / magnitude, 0, vec.z / magnitude);
}

double calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return std::sqrt(std::pow(p2.x - p1.x, 2) +  std::pow(p2.z - p1.z, 2));
}

pcl::PointXYZ calculateStep(const pcl::PointXYZ& source, const Forces& dir_forces, double step_size) {
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
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& path_point : path) {
        double distance = calculateDistance(point, path_point);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    return min_distance;
}

cv::Mat drawFloorVector(std::vector<VertexType>& vertices, std::vector<std::vector<int>>& plane_vertices,         
        int groundIdx, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, Plane plane, cv::Mat image){

        cv::Mat output_image = image.clone();
        
        pcl::PointXYZ source = pcl::PointXYZ(0,0,0);
        pcl::PointXYZ destination = pcl::PointXYZ(0,0,1000);

        double step = 10; // step size is 10 cm
        double distanceThreshold = 100; // want to be within 100 cm of destination

        std::vector<pcl::PointXYZ> tracedPath;

        double previousDistance = calculateDistance(source, destination);
        int tolerance=10;

        tracedPath.push_back(source);

        // continue loop until we reach the end position
        while (true){

            Forces forces = resultantForces(cloud, source, destination);
            pcl::PointXYZ new_point = calculateStep(source, forces, step);

            source.x=new_point.x;
            source.z=new_point.z;

            tracedPath.push_back(new_point); // add point

            std::cout << "New point: (" << new_point.x << ", " << new_point.y << ", " << new_point.z << ")\n";

            double currentDistance = calculateDistance(source, destination);

            if (currentDistance < distanceThreshold) {
                std::cout << "Reached the destination.\n";
                break;
            }
            if (currentDistance >= previousDistance) {
                tolerance--;
                if (!tolerance){
                    std::cout << "No longer approaching the destination. Exiting...\n";
                    break;
                }
            }else{
                tolerance=10;
            }

            previousDistance = currentDistance;

        }

        std::vector<double> distances;
        for (const auto& idx : plane_vertices[groundIdx]){
            VertexType vertex = vertices[idx];
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

            // from plane_detection.h:44 -> const int pixIdx = row * w + col;
            int row = idx/image.cols;
            int col = idx % image.cols;
            groundPts[i] = std::make_tuple(row, col); 
        }
        
        for (size_t i=0; i<nGround; i++){
            std::tuple<int,int> tup = groundPts[i];
            int row = std::get<0>(tup);
            int col = std::get<1>(tup);
            // Blend the original image pixel with the mask pixel using alpha
            cv::Vec3b originalPixel = image.at<cv::Vec3b>(row, col);
            cv::Vec3b maskPixel = rgbVals[i];

            // Alpha blending
            float alpha = 0.5;
            cv::Vec3b blendedPixel;
            for (int j = 0; j < 3; j++) {
                blendedPixel[j] = static_cast<uchar>(alpha * maskPixel[j] + (1 - alpha) * originalPixel[j]);
            }

            output_image.at<cv::Vec3b>(row, col) = blendedPixel;
        }

        // Draw x and z values of a few ground points on the image
        for (size_t i = 0; i < std::min(nGround, static_cast<size_t>(640*5*10)); i+=640*5){ // Draw for the first 10 points
            std::tuple<int, int> tup = groundPts[i];
            int row = std::get<0>(tup);
            int col = std::get<1>(tup);

            // VertexType vertex = vertices[plane_vertices[groundIdx][i]];
            std::string text = "(" + std::to_string(distances[i]) + ")";
            
            cv::putText(output_image, text, cv::Point(col, row), cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(255, 255, 255), 1);
        }

        return output_image;
}

#endif // PLANE_OPERATIONS_H
