#ifndef CAR_MODEL_H
#define CAR_MODEL_H

#include "raylib.h"
#include "raymath.h"
#include <cmath>

// Car Model Parameters
class CarModel {
public:
    CarModel();
    void Update(float deltaTime, bool throttlePressed, bool brakePressed, bool steerLeftPressed, bool steerRightPressed);
    void Reset();
    
    // Position and orientation getters
    Vector3 GetPosition() const { return position; }
    float GetHeading() const { return heading; }
    float GetSteeringAngle() const { return steeringAngle; }
    float GetWheelRotation(float deltaTime);
    
    // Performance stats
    float GetVelocity() const { return velocity; }
    float GetEngineRPM() const { return engineRPM; }
    float GetThrottleInput() const { return throttleInput; }
    float GetLastAppliedForce() const { return lastAppliedForce; }
    float GetYawRate() const { return yawRate; }
    
private:
    // Engine torque model (simplified)
    float GetEngineTorque(float rpm, float throttle);
    
    // Tire slip curve (simplified Pacejka)
    float GetTireForce(float slipRatio, float normalForce);
    
    // Position and orientation
    Vector3 position = { 0.0f, 0.0f, 0.0f };
    float heading = PI / 2.0f; // Initialize heading to 90 degrees (forward)
    
    // Physical properties
    float mass = 1200.0f; // kg
    float wheelbase = 2.5f; // Distance between front and rear axles (m)
    float trackWidth = 1.6f; // Width between left and right wheels (m)
    float CoG_height = 0.5f; // Center of gravity height (m)
    
    // Velocities
    float velocity = 0.0f; // Current forward velocity (m/s)
    float lateralVelocity = 0.0f; // Sideways velocity (m/s)
    float yawRate = 0.0f; // Rotation rate around vertical axis (rad/s)
    
    // Debug
    float lastAppliedForce = 0.0f; // For debugging
    
    // Engine properties
    float engineRPM = 800.0f; // Current engine RPM
    float idleRPM = 800.0f; // Idle RPM
    float maxRPM = 7000.0f; // Maximum RPM
    float redlineRPM = 6500.0f; // Redline RPM
    
    // Transmission
    float gearRatio = 8.0f; // Single fixed gear ratio
    float differentialRatio = 3.5f; // Final drive ratio
    float transmissionEfficiency = 0.9f; // Drivetrain efficiency
    
    // Wheels and tires
    float wheelRadius = 0.3f; // meters
    float wheelMomentOfInertia = 1.5f; // kg*m^2
    float wheelAngularVelocity = 0.0f; // rad/s
    
    // Tire model parameters
    float tireFrictionCoefficient = 1.0f; // Base friction
    float corneringStiffness = 10000.0f; // Cornering stiffness coefficient (N/rad)
    
    // Controls
    float throttleInput = 0.0f; // 0 to 1
    float brakeInput = 0.0f; // 0 to 1
    float steeringAngle = 0.0f; // Current steering angle in radians
    float maxSteeringAngle = PI / 4.0f; // Maximum steering angle (45 degrees)
    float steeringSpeed = PI / 4.0f; // Steering speed in radians per second
    
    // Forces
    float engineTorque = 0.0f; // Current engine torque
    float wheelTorque = 0.0f; // Torque at the wheels
    float tractionForce = 0.0f; // Force pushing car forward
    float dragForce = 0.0f; // Aerodynamic drag force
    
    // Constants
    float dragCoefficient = 0.3f; // Aerodynamic drag coefficient
    float rollingResistance = 0.015f; // Rolling resistance coefficient
    float brakeForceMax = 12000.0f; // Maximum brake force in Newtons
};

#endif // CAR_MODEL_H