#include "../include/model_factory.h"

ModelFactory::ModelFactory() {
    // Default constructor - models will be initialized separately
}

ModelFactory::~ModelFactory() {
    // Resources should be unloaded explicitly by calling Unload()
}

void ModelFactory::Initialize() {
    // Create car parts
    carBody = CreateBoxModel(2.0f, 0.4f, 4.2f, BLUE);
    carHood = CreateBoxModel(1.9f, 0.2f, 1.0f, DARKBLUE);
    carTrunk = CreateBoxModel(1.9f, 0.25f, 0.8f, DARKBLUE);
    carCabin = CreateBoxModel(1.8f, 0.7f, 2.2f, SKYBLUE);
    carRoof = CreateBoxModel(1.7f, 0.15f, 2.0f, DARKBLUE);
    
    // Create wheels
    wheelModel = CreateWheelModel();
    
    // Create headlights
    headlight = LoadModelFromMesh(GenMeshSphere(0.15f, 16, 16));
    
    // Create taillights
    taillight = CreateBoxModel(0.3f, 0.1f, 0.05f, RED);
    
    // Create bumpers
    bumper = CreateBoxModel(2.0f, 0.3f, 0.2f, BLACK);
}

void ModelFactory::Unload() {
    // De-Initialize all models to prevent memory leaks
    UnloadModel(carBody);
    UnloadModel(carHood);
    UnloadModel(carTrunk);
    UnloadModel(carCabin);
    UnloadModel(carRoof);
    UnloadModel(wheelModel);
    UnloadModel(headlight);
    UnloadModel(taillight);
    UnloadModel(bumper);
}

Model ModelFactory::CreateWheelModel() {
    // Create a wheel that looks like a cylinder
    Mesh wheelMesh = GenMeshCylinder(0.4f, 0.3f, 12);
    Model wheelModel = LoadModelFromMesh(wheelMesh);
    
    // Rotate the wheel to match the orientation in the Three.js version
    wheelModel.transform = MatrixRotateZ(PI / 2.0f);
    
    return wheelModel;
}

Model ModelFactory::CreateBoxModel(float width, float height, float length, Color color) {
    Mesh boxMesh = GenMeshCube(width, height, length);
    Model boxModel = LoadModelFromMesh(boxMesh);
    boxModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
    return boxModel;
}