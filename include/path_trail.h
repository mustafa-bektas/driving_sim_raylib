#ifndef PATH_TRAIL_H
#define PATH_TRAIL_H

#include "raylib.h"
#include "raymath.h"
#include <vector>

// Path visualization
class PathTrail {
public:
    PathTrail(int maxSize = 500);
    void AddPoint(Vector3 position);
    void Draw();
    void Clear();
    
private:
    std::vector<Vector3> points;
    int maxPoints;
};

#endif // PATH_TRAIL_H