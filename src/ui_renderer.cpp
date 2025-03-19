#include "../include/ui_renderer.h"
#include <cstdio>

void UIRenderer::RenderUI(const CarModel& car) {
    // Draw UI background
    DrawRectangle(10, 10, 300, 220, Fade(BLACK, 0.7f));
    DrawText("Dynamic Engine Car Simulation", 20, 20, 20, WHITE);
    
    // Draw controls info
    DrawText("Controls:", 20, 50, 10, WHITE);
    DrawText("- Up/Down Arrow or W/S: Throttle/Brake", 20, 70, 10, WHITE);
    DrawText("- Left/Right Arrow or A/D: Steer", 20, 90, 10, WHITE);
    DrawText("- R: Reset position", 20, 110, 10, WHITE);
    
    // Display car stats
    char speedText[64];
    sprintf(speedText, "Speed: %.2f m/s", fabsf(car.GetVelocity()));
    DrawText(speedText, 20, 140, 10, WHITE);
    
    char rpmText[64];
    sprintf(rpmText, "RPM: %d", (int)car.GetEngineRPM());
    DrawText(rpmText, 20, 160, 10, WHITE);
    
    DrawText("Gear: 1 (Fixed)", 20, 180, 10, WHITE);
    
    char throttleText[64];
    sprintf(throttleText, "Throttle: %d%%", (int)(car.GetThrottleInput() * 100.0f));
    DrawText(throttleText, 20, 200, 10, WHITE);
    
    char steeringText[64];
    sprintf(steeringText, "Steering Angle: %.2f°", car.GetSteeringAngle() * (180.0f / PI));
    DrawText(steeringText, 20, 220, 10, WHITE);
    
    char forceText[64];
    sprintf(forceText, "Traction Force: %d N", (int)car.GetLastAppliedForce());
    DrawText(forceText, 20, 240, 10, WHITE);
    
    char yawText[64];
    sprintf(yawText, "Yaw Rate: %.2f rad/s", car.GetYawRate());
    DrawText(yawText, 20, 260, 10, WHITE);
}