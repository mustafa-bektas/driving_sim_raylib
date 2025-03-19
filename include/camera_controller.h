#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include "raylib.h"
#include "raymath.h"
#include "car_model.h"

class CameraController {
public:
    CameraController();
    void InitCamera();
    void UpdateCamera(const CarModel& car, float deltaTime);
    Camera& GetCamera() { return camera; }

private:
    Camera camera;
    Vector3 cameraTargetPosition;
    Vector3 cameraLookAt;
    
    // Camera settings
    float distance = 10.0f;      // Distance behind the car
    float height = 4.0f;         // Height above the car
    float lookAheadDistance = 3.0f; // Distance to look ahead
};

#endif // CAMERA_CONTROLLER_H