#include "../include/path_trail.h"

PathTrail::PathTrail(int maxSize) : maxPoints(maxSize) {
    points.reserve(maxSize);
}

void PathTrail::AddPoint(Vector3 position) {
    // Add current position to path if it's far enough from the last point
    if (points.empty() || Vector3Distance(points.back(), position) > 0.5f) {
        points.push_back(position);
        
        // Limit path length
        if (points.size() > maxPoints) {
            points.erase(points.begin());
        }
    }
}

void PathTrail::Draw() {
    for (size_t i = 0; i < points.size() - 1; i++) {
        DrawLine3D(points[i], points[i + 1], RED);
    }
}

void PathTrail::Clear() {
    points.clear();
}