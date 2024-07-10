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
    std::cout << Dmax << std::endl;
    const double lambda = 0.5;
    const double hat = 5000;
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
    std::cout << dest.z << std::endl;

    double force_z_attr = std::cos(phi)*hat/calcModulus(dest.x-source.x, dest.z-source.z);
    double force_x_attr = std::sin(phi)*hat/calcModulus(dest.x-source.x, dest.z-source.z);
    Forces forces;
    forces.x = force_x_attr - force_x_repulse;
    forces.z = force_z_attr - force_z_repulse;
    std::cout << "force x: " << forces.x << ", forces z: " << forces.z << std::endl;
    return forces;
}


#endif // PLANE_OPERATIONS_H
