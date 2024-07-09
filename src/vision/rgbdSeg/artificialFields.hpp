#ifndef ARTIFICIAL_FIELDS_H
#define ARTIFICIAL_FIELDS_H

#include <memory>
#include <cmath>
#include "plane_detection.h"

#define C_ATTRACT 10.
#define C_REPULSE 5.

// Forward declaration
struct Plane {
    double a, b, c, d;

    Plane(double _a, double _b, double _c, double _d)
        : a(_a), b(_b), c(_c), d(_d) {
    }
};

Plane computePlaneEq(std::vector<ahc::PlaneSeg::shared_ptr> planeSegs, int groundIdx) {
    Plane plane(0,0,0,0);
    std::cout << groundIdx << std::endl;
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

std::vector<int> mapPointsToSurfaces(const std::vector<VertexType> points, const std::vector<std::vector<int>> surfaces) {
    std::vector<int> surfaceIndices(points.size(), -1); // Initialize with -1 for points not found in any surface
    // Iterate over each surface
    for (size_t surfaceIdx = 0; surfaceIdx < surfaces.size(); ++surfaceIdx) {
        std::vector<int> surface = surfaces[surfaceIdx];
        
        // Assign the surface index to each point in the current surface
        for (int pointIdx : surface) {
            if (pointIdx < static_cast<int>(points.size())) { // Ensure pointIdx is within valid range
                surfaceIndices[pointIdx] = static_cast<int>(surfaceIdx);
            }
        }
    }

    return surfaceIndices;
}

cv::Mat draw2DPoints(std::vector<VertexType> points, std::vector<std::vector<int>> planeVertices, int groundIdx){
    std::vector<int> surfaceIndices = mapPointsToSurfaces(points, planeVertices);

    // Step 1: Find minimum and maximum x and y values
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::min();

    for (const auto& point : points) {
        if (point.x() < minX) minX = point.x();
        if (point.x() > maxX) maxX = point.x();
        if (point.y() < minY) minY = point.y();
        if (point.y() > maxY) maxY = point.y();
    }

    // Step 2: Scale points to range [0, 500]
    for (auto& point : points) {
        point.x() = static_cast<int>(500.0 * (point.x() - minX) / (maxX - minX));
        point.y() = static_cast<int>(500.0 * (point.y() - minY) / (maxY - minY));
    }

    // Step 3: Create a blank image
    cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // Step 4: Draw scaled points on the image
    for (size_t i = 0; i < points.size(); ++i) {  // Use size_t for the loop variable
        const VertexType& point = points[i];
        cv::Scalar color(0, 0, 255); // Default color: Red

        // Change color if point belongs to the ground index
        if (surfaceIndices[i] == groundIdx) {
            color = cv::Scalar(0, 255, 0); // Green color for ground index
        }

        // Draw the point on the image
        cv::circle(image, cv::Point(static_cast<int>(point.x()), static_cast<int>(point.y())), 5, color, -1);
    }
    return image;
}


#endif // PLANE_OPERATIONS_H
