#include "raylib.h"
#include <cmath>
#include <string>

#include "../include/car_model.h"
#include "../include/path_trail.h"
#include "../include/keyboard_state.h"
#include "../include/camera_controller.h"
#include "../include/model_factory.h"
#include "../include/ui_renderer.h"
#include "../include/car_renderer.h"

int main() {
    // Initialization
    const int screenWidth = 1280;
    const int screenHeight = 720;
    
    InitWindow(screenWidth, screenHeight, "Drivable Car with Dynamic Model in Raylib");
    
    // Create objects
    CarModel carModel;
    PathTrail pathTrail;
    KeyboardState keys;
    CameraController cameraController;
    ModelFactory modelFactory;
    UIRenderer uiRenderer;
    
    // Initialize camera
    cameraController.InitCamera();
    
    // Initialize models
    modelFactory.Initialize();
    
    // Create car renderer
    CarRenderer carRenderer(modelFactory);
    
    // Timing variables
    double previousTime = GetTime();
    float wheelRotation = 0.0f;
    
    SetTargetFPS(60);  // Set our game to run at 60 frames-per-second
    
    // Main game loop
    while (!WindowShouldClose()) {  // Detect window close button or ESC key
        // Calculate delta time
        double currentTime = GetTime();
        float deltaTime = (float)(currentTime - previousTime);
        previousTime = currentTime;
        deltaTime = fmin(deltaTime, 0.1f); // Cap delta time to prevent physics issues
        
        // Update keyboard state
        keys.Update();
        
        // Handle reset
        if (keys.IsResetPressed()) {
            carModel.Reset();
            pathTrail.Clear();
        }
        
        // Update car physics
        carModel.Update(deltaTime, keys.IsThrottlePressed(), keys.IsBrakePressed(), 
                       keys.IsSteeringLeftPressed(), keys.IsSteeringRightPressed());
        
        // Update wheel rotation for visuals
        wheelRotation = carModel.GetWheelRotation(deltaTime);
        
        // Update path trail
        pathTrail.AddPoint(carModel.GetPosition());
        
        // Update camera
        cameraController.UpdateCamera(carModel, deltaTime);
        
        // Start drawing
        BeginDrawing();
            ClearBackground(SKYBLUE);
            
            BeginMode3D(cameraController.GetCamera());
                // Draw ground plane
                DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 100.0f, 100.0f }, DARKGREEN);
                
                // Draw grid for better visual reference
                DrawGrid(100, 1.0f);
                
                // Draw path trail
                pathTrail.Draw();
                
                // Draw car
                carRenderer.Render(carModel, wheelRotation);
            EndMode3D();
            
            // Draw UI
            uiRenderer.RenderUI(carModel);
            
        EndDrawing();
    }
    
    // De-Initialization
    modelFactory.Unload();
    
    CloseWindow();  // Close window and OpenGL context
    
    return 0;
}