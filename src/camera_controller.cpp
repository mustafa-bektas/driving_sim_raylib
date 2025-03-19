#include "../include/camera_controller.h"

CameraController::CameraController() {
    // Initialize camera position
    cameraTargetPosition = { 0.0f, 0.0f, 0.0f };
    cameraLookAt = { 0.0f, 0.0f, 0.0f };
}

void CameraController::InitCamera() {
    // Set initial camera values
    camera.position = (Vector3){ 0.0f, 7.0f, 15.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

void CameraController::UpdateCamera(const CarModel& car, float deltaTime) {
    // Get car position and heading
    Vector3 carPosition = car.GetPosition();
    float carHeading = car.GetHeading();
    float carVelocity = car.GetVelocity();
    
    // Calculate ideal camera position based on car's heading
    Vector3 cameraOffset = {
        sinf(carHeading) * distance,
        height,
        -cosf(carHeading) * distance
    };
    
    // Calculate look-at position (slightly ahead of the car)
    Vector3 lookAtOffset = {
        -sinf(carHeading) * lookAheadDistance,
        0.0f,
        cosf(carHeading) * lookAheadDistance
    };
    Vector3 lookAtPosition = Vector3Add(carPosition, lookAtOffset);
    
    // Calculate camera smoothing factor based on speed
    // Faster speed = quicker camera response
    const float velocityFactor = fmin(fabsf(carVelocity) / 5.0f, 1.0f);
    const float smoothFactor = 0.03f + velocityFactor * 0.07f;
    
    // Smoothly transition camera position
    cameraTargetPosition = Vector3Add(carPosition, cameraOffset);
    camera.position = Vector3Lerp(camera.position, cameraTargetPosition, smoothFactor);
    
    // Smoothly transition look-at target
    cameraLookAt = Vector3Lerp(cameraLookAt, lookAtPosition, smoothFactor);
    camera.target = cameraLookAt;
}