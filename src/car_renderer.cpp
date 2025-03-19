#include "../include/car_renderer.h"

CarRenderer::CarRenderer(const ModelFactory& modelFactory) 
    : models(modelFactory) {
}

void CarRenderer::Render(const CarModel& car, float wheelRotation) {
    // Get car position and heading
    Vector3 position = car.GetPosition();
    float heading = car.GetHeading();
    float steeringAngle = car.GetSteeringAngle();
    
    // Set car transform matrix
    Matrix carTransform = MatrixIdentity();
    carTransform = MatrixMultiply(carTransform, MatrixRotateY(-heading));
    carTransform = MatrixMultiply(carTransform, MatrixTranslate(position.x, 0.4f, position.z));
    
    // Draw car body
    DrawModelEx(models.GetCarBodyModel(), Vector3Zero(), (Vector3){ 0.0f, 1.0f, 0.0f }, 
                0.0f, (Vector3){ 1.0f, 1.0f, 1.0f }, BLUE);
    
    // Draw car hood
    Matrix hoodMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.7f, 1.5f), carTransform);
    DrawMesh(models.GetCarHoodModel().meshes[0], models.GetCarHoodModel().materials[0], hoodMatrix);
    
    // Draw car trunk
    Matrix trunkMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.7f, -1.6f), carTransform);
    DrawMesh(models.GetCarTrunkModel().meshes[0], models.GetCarTrunkModel().materials[0], trunkMatrix);
    
    // Draw car cabin
    Matrix cabinMatrix = MatrixMultiply(MatrixTranslate(0.0f, 1.15f, 0.0f), carTransform);
    DrawMesh(models.GetCarCabinModel().meshes[0], models.GetCarCabinModel().materials[0], cabinMatrix);
    
    // Draw car roof
    Matrix roofMatrix = MatrixMultiply(MatrixTranslate(0.0f, 1.55f, 0.0f), carTransform);
    DrawMesh(models.GetCarRoofModel().meshes[0], models.GetCarRoofModel().materials[0], roofMatrix);
    
    // Draw wheels
    // Rear left wheel
    Matrix wheelRLMatrix = MatrixIdentity();
    wheelRLMatrix = MatrixMultiply(wheelRLMatrix, MatrixRotateX(wheelRotation));
    wheelRLMatrix = MatrixMultiply(wheelRLMatrix, MatrixTranslate(-1.05f, 0.4f, -1.5f));
    wheelRLMatrix = MatrixMultiply(wheelRLMatrix, carTransform);
    DrawMesh(models.GetWheelModel().meshes[0], models.GetWheelModel().materials[0], wheelRLMatrix);
    
    // Rear right wheel
    Matrix wheelRRMatrix = MatrixIdentity();
    wheelRRMatrix = MatrixMultiply(wheelRRMatrix, MatrixRotateX(wheelRotation));
    wheelRRMatrix = MatrixMultiply(wheelRRMatrix, MatrixTranslate(1.05f, 0.4f, -1.5f));
    wheelRRMatrix = MatrixMultiply(wheelRRMatrix, carTransform);
    DrawMesh(models.GetWheelModel().meshes[0], models.GetWheelModel().materials[0], wheelRRMatrix);
    
    // Front left wheel (with steering)
    Matrix wheelFLMatrix = MatrixIdentity();
    wheelFLMatrix = MatrixMultiply(wheelFLMatrix, MatrixRotateX(wheelRotation));
    wheelFLMatrix = MatrixMultiply(wheelFLMatrix, MatrixRotateY(-steeringAngle));
    wheelFLMatrix = MatrixMultiply(wheelFLMatrix, MatrixTranslate(-1.05f, 0.4f, 1.5f));
    wheelFLMatrix = MatrixMultiply(wheelFLMatrix, carTransform);
    DrawMesh(models.GetWheelModel().meshes[0], models.GetWheelModel().materials[0], wheelFLMatrix);
    
    // Front right wheel (with steering)
    Matrix wheelFRMatrix = MatrixIdentity();
    wheelFRMatrix = MatrixMultiply(wheelFRMatrix, MatrixRotateX(wheelRotation));
    wheelFRMatrix = MatrixMultiply(wheelFRMatrix, MatrixRotateY(-steeringAngle));
    wheelFRMatrix = MatrixMultiply(wheelFRMatrix, MatrixTranslate(1.05f, 0.4f, 1.5f));
    wheelFRMatrix = MatrixMultiply(wheelFRMatrix, carTransform);
    DrawMesh(models.GetWheelModel().meshes[0], models.GetWheelModel().materials[0], wheelFRMatrix);
    
    // Draw headlights
    Matrix headlightLeftMatrix = MatrixMultiply(MatrixTranslate(-0.6f, 0.5f, 2.1f), carTransform);
    DrawMesh(models.GetHeadlightModel().meshes[0], models.GetHeadlightModel().materials[0], headlightLeftMatrix);
    
    Matrix headlightRightMatrix = MatrixMultiply(MatrixTranslate(0.6f, 0.5f, 2.1f), carTransform);
    DrawMesh(models.GetHeadlightModel().meshes[0], models.GetHeadlightModel().materials[0], headlightRightMatrix);
    
    // Draw taillights
    Matrix taillightLeftMatrix = MatrixMultiply(MatrixTranslate(-0.7f, 0.6f, -2.1f), carTransform);
    DrawMesh(models.GetTaillightModel().meshes[0], models.GetTaillightModel().materials[0], taillightLeftMatrix);
    
    Matrix taillightRightMatrix = MatrixMultiply(MatrixTranslate(0.7f, 0.6f, -2.1f), carTransform);
    DrawMesh(models.GetTaillightModel().meshes[0], models.GetTaillightModel().materials[0], taillightRightMatrix);
    
    // Draw bumpers
    Matrix frontBumperMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.35f, 2.1f), carTransform);
    DrawMesh(models.GetBumperModel().meshes[0], models.GetBumperModel().materials[0], frontBumperMatrix);
    
    Matrix rearBumperMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.35f, -2.1f), carTransform);
    DrawMesh(models.GetBumperModel().meshes[0], models.GetBumperModel().materials[0], rearBumperMatrix);
}