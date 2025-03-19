#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <string>

// Car Model Parameters
struct CarModel {
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
    
    // Engine torque model (simplified)
    float GetEngineTorque(float rpm, float throttle) {
        // Simple torque curve: peak at 4000 RPM
        float normalizedRPM = rpm / maxRPM;
        
        // Create a simple torque curve that peaks at mid RPM range
        // Modified to provide more torque at low RPM for better starts
        float torqueFactor = 0.5f + (3.5f * normalizedRPM * (1.0f - normalizedRPM));
        
        // Maximum torque in Nm
        const float maxTorque = 350.0f;
        
        // Apply throttle and torque curve
        return maxTorque * torqueFactor * throttle;
    }
    
    // Tire slip curve (simplified Pacejka)
    float GetTireForce(float slipRatio, float normalForce) {
        // Simplified tire force calculation using slip ratio
        // Based on a simplified Pacejka "Magic Formula"
        const float B = 10.0f; // Stiffness factor
        const float C = 1.9f; // Shape factor
        const float D = tireFrictionCoefficient * normalForce; // Peak value
        
        // Saturate slip ratio to prevent extreme values
        const float safeSlip = fmax(-1.0f, fmin(1.0f, slipRatio));
        
        // Calculate force using simplified magic formula
        return D * sin(C * atan(B * safeSlip));
    }
};

// Path visualization
struct PathTrail {
    std::vector<Vector3> points;
    int maxPoints = 500;
    
    void AddPoint(Vector3 position) {
        // Add current position to path
        if (points.empty() || 
            Vector3Distance(points.back(), position) > 0.5f) {
            points.push_back(position);
            
            // Limit path length
            if (points.size() > maxPoints) {
                points.erase(points.begin());
            }
        }
    }
    
    void Draw() {
        for (size_t i = 0; i < points.size() - 1; i++) {
            DrawLine3D(points[i], points[i + 1], RED);
        }
    }
};

// Keyboard input handler
struct KeyboardState {
    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;
    bool w = false;
    bool s = false;
    bool a = false;
    bool d = false;
    bool r = false;
    
    void Update() {
        up = IsKeyDown(KEY_UP);
        down = IsKeyDown(KEY_DOWN);
        left = IsKeyDown(KEY_LEFT);
        right = IsKeyDown(KEY_RIGHT);
        w = IsKeyDown(KEY_W);
        s = IsKeyDown(KEY_S);
        a = IsKeyDown(KEY_A);
        d = IsKeyDown(KEY_D);
        r = IsKeyDown(KEY_R);
    }
};

// Function to create a wheel mesh
Model CreateWheelModel() {
    // Create a wheel that looks like a cylinder
    Mesh wheelMesh = GenMeshCylinder(0.4f, 0.3f, 12);
    Model wheelModel = LoadModelFromMesh(wheelMesh);
    
    // Rotate the wheel to match the orientation in the Three.js version
    wheelModel.transform = MatrixRotateZ(PI / 2.0f);
    
    return wheelModel;
}

// Function to create a simple box model with a given color
Model CreateBoxModel(float width, float height, float length, Color color) {
    Mesh boxMesh = GenMeshCube(width, height, length);
    Model boxModel = LoadModelFromMesh(boxMesh);
    boxModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = color;
    return boxModel;
}

int main() {
    // Initialization
    const int screenWidth = 1280;
    const int screenHeight = 720;
    
    InitWindow(screenWidth, screenHeight, "Drivable Car with Dynamic Model in Raylib");
    
    // Set our camera
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 7.0f, 15.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    
    // Camera smooth parameters
    Vector3 cameraTargetPosition = { 0.0f, 0.0f, 0.0f };
    
    // Vector3 to keep track of camera look-at position for smooth transitions
    Vector3 cameraLookAt = { 0.0f, 0.0f, 0.0f };
    
    // Car physics model
    CarModel carModel;
    
    // Create visual models for car parts
    Model carBody = CreateBoxModel(2.0f, 0.4f, 4.2f, BLUE);
    Model carHood = CreateBoxModel(1.9f, 0.2f, 1.0f, DARKBLUE);
    Model carTrunk = CreateBoxModel(1.9f, 0.25f, 0.8f, DARKBLUE);
    Model carCabin = CreateBoxModel(1.8f, 0.7f, 2.2f, SKYBLUE);
    Model carRoof = CreateBoxModel(1.7f, 0.15f, 2.0f, DARKBLUE);
    
    // Create wheels
    Model wheelModel = CreateWheelModel();
    
    // Create headlights
    Model headlight = LoadModelFromMesh(GenMeshSphere(0.15f, 16, 16));
    
    // Create taillights
    Model taillight = CreateBoxModel(0.3f, 0.1f, 0.05f, RED);
    
    // Create bumpers
    Model bumper = CreateBoxModel(2.0f, 0.3f, 0.2f, BLACK);
    
    // Create path trail
    PathTrail pathTrail;
    
    // Keyboard state
    KeyboardState keys;
    
    // Function to reset car position and state
    auto resetCar = [&]() {
        // Reset position and orientation
        carModel.position = (Vector3){ 0.0f, 0.0f, 0.0f };
        carModel.heading = PI / 2.0f; // Forward
        
        // Reset physics state
        carModel.velocity = 0.0f;
        carModel.lateralVelocity = 0.0f;
        carModel.yawRate = 0.0f;
        carModel.engineRPM = carModel.idleRPM;
        carModel.wheelAngularVelocity = 0.0f;
        carModel.throttleInput = 0.0f;
        carModel.brakeInput = 0.0f;
        carModel.steeringAngle = 0.0f;
        
        // Clear path trail
        pathTrail.points.clear();
    };
    
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
        if (keys.r) {
            resetCar();
        }
        
        // Handle input
        // Throttle input (accelerator pedal)
        if (keys.up || keys.w) {
            carModel.throttleInput += 2.0f * deltaTime; // Smoother throttle application
            carModel.throttleInput = fmin(carModel.throttleInput, 1.0f);
            carModel.brakeInput = 0.0f; // Release brakes when accelerating
        } else {
            carModel.throttleInput -= 3.0f * deltaTime; // Throttle returns to zero when released
            carModel.throttleInput = fmax(carModel.throttleInput, 0.0f);
        }
        
        // Brake input
        if (keys.down || keys.s) {
            carModel.brakeInput += 3.0f * deltaTime; // Smoother brake application
            carModel.brakeInput = fmin(carModel.brakeInput, 1.0f);
            carModel.throttleInput = 0.0f; // Release throttle when braking
        } else {
            carModel.brakeInput -= 4.0f * deltaTime; // Brakes release quickly
            carModel.brakeInput = fmax(carModel.brakeInput, 0.0f);
        }
        
        // Steering input
        if (keys.left || keys.a) {
            carModel.steeringAngle -= carModel.steeringSpeed * deltaTime;
            carModel.steeringAngle = fmax(carModel.steeringAngle, -carModel.maxSteeringAngle);
        } else if (keys.right || keys.d) {
            carModel.steeringAngle += carModel.steeringSpeed * deltaTime;
            carModel.steeringAngle = fmin(carModel.steeringAngle, carModel.maxSteeringAngle);
        } else {
            // Return steering to center gradually
            if (fabsf(carModel.steeringAngle) > 0.01f) {
                carModel.steeringAngle -= (carModel.steeringAngle > 0 ? 1.0f : -1.0f) * carModel.steeringSpeed * 0.5f * deltaTime;
            } else {
                carModel.steeringAngle = 0.0f;
            }
        }
        
        // Update car physics
        // Calculate weight distribution (simplified)
        const float totalWeight = carModel.mass * 9.81f; // N (mass * gravity)
        const float rearAxleWeight = totalWeight * 0.5f; // Simplified 50/50 weight distribution
        
        // Calculate engine torque based on throttle and RPM
        carModel.engineTorque = carModel.GetEngineTorque(carModel.engineRPM, carModel.throttleInput);
        
        // Calculate wheel torque through transmission
        const float totalGearRatio = carModel.gearRatio * carModel.differentialRatio;
        carModel.wheelTorque = carModel.engineTorque * totalGearRatio * carModel.transmissionEfficiency;
        
        // Add "clutch engagement" effect to help car start moving from standstill
        if (carModel.throttleInput > 0.0f && fabsf(carModel.velocity) < 0.5f) {
            carModel.wheelAngularVelocity += carModel.throttleInput * 5.0f * deltaTime;
        }
        
        // Calculate tractive force at driven wheels
        carModel.tractionForce = carModel.wheelTorque / carModel.wheelRadius;
        
        // Calculate longitudinal slip ratio
        const float wheelSpeed = carModel.wheelAngularVelocity * carModel.wheelRadius;
        const float speedDiff = wheelSpeed - carModel.velocity;
        
        // Special case for starting from standstill
        float tireSlipRatio;
        if (fabsf(carModel.velocity) < 0.1f && carModel.throttleInput > 0.0f) {
            tireSlipRatio = 0.1f; // Small positive slip to get the car moving
        } else {
            // Prevent division by zero when calculating slip
            float denominator = fmax(fmax(fabsf(carModel.velocity), fabsf(wheelSpeed)), 0.1f);
            tireSlipRatio = speedDiff / denominator;
        }
        
        // Calculate longitudinal tire force using slip model
        const float longitudinalForce = carModel.GetTireForce(tireSlipRatio, rearAxleWeight);
        
        // Apply traction force limit based on tire grip
        float limitedTractionForce = fmin(
            fabsf(carModel.tractionForce),
            fabsf(longitudinalForce)
        ) * (carModel.tractionForce >= 0.0f ? 1.0f : -1.0f);
        
        // Add a minimum starting force when throttle is applied to simulate clutch engagement
        if (carModel.throttleInput > 0.05f && fabsf(carModel.velocity) < 0.5f) {
            const float startingForce = 1000.0f * carModel.throttleInput;
            limitedTractionForce = fmax(limitedTractionForce, startingForce);
        }
        
        // Calculate drag force (increases with square of velocity)
        carModel.dragForce = 0.5f * carModel.dragCoefficient * carModel.velocity * fabsf(carModel.velocity);
        
        // Calculate rolling resistance
        const float rollingForce = carModel.rollingResistance * totalWeight * (carModel.velocity >= 0.0f ? 1.0f : -1.0f);
        
        // Calculate braking force
        const float brakeForce = carModel.brakeInput * carModel.brakeForceMax * (carModel.velocity <= 0.0f ? 1.0f : -1.0f);
        
        // Calculate net longitudinal force
        const float netLongitudinalForce = limitedTractionForce - carModel.dragForce - rollingForce + brakeForce;
        carModel.lastAppliedForce = netLongitudinalForce; // Store for debugging
        
        // Calculate longitudinal acceleration
        const float longitudinalAcceleration = netLongitudinalForce / carModel.mass;
        
        // Calculate effective steering angle with speed-dependent adjustment
        const float speedFactor = fmin(1.0f, fabsf(carModel.velocity) / 5.0f);
        const float effectiveSteeringAngle = carModel.steeringAngle * (1.0f - 0.3f * speedFactor);
        
        // Bicycle model calculations for slip angles
        // For front wheels (steering wheels)
        const float frontSlipAngle = atan2f(
            carModel.lateralVelocity + carModel.yawRate * (carModel.wheelbase/2.0f), 
            fabsf(carModel.velocity) + 0.001f
        ) - effectiveSteeringAngle;
        
        // For rear wheels (non-steering)
        const float rearSlipAngle = atan2f(
            carModel.lateralVelocity - carModel.yawRate * (carModel.wheelbase/2.0f), 
            fabsf(carModel.velocity) + 0.001f
        );
        
        // Calculate forces using slip angles and the cornering stiffness
        const float frontLateralForce = -carModel.corneringStiffness * frontSlipAngle;
        const float rearLateralForce = -carModel.corneringStiffness * rearSlipAngle;
        
        // Limit lateral forces based on weight and friction
        const float maxFrontLateralForce = totalWeight * 0.5f * carModel.tireFrictionCoefficient;
        const float maxRearLateralForce = totalWeight * 0.5f * carModel.tireFrictionCoefficient;
        
        const float limitedFrontLateralForce = fmax(-maxFrontLateralForce, 
                                                fmin(maxFrontLateralForce, frontLateralForce));
        const float limitedRearLateralForce = fmax(-maxRearLateralForce, 
                                               fmin(maxRearLateralForce, rearLateralForce));
        
        // Calculate total lateral force
        const float totalLateralForce = limitedFrontLateralForce + limitedRearLateralForce;
        
        // Calculate lateral acceleration
        const float lateralAcceleration = totalLateralForce / carModel.mass;
        
        // Calculate yaw moment from steering forces
        // Positive moment turns clockwise
        const float yawMoment = (limitedFrontLateralForce * (carModel.wheelbase/2.0f)) - 
                             (limitedRearLateralForce * (carModel.wheelbase/2.0f));
        
        // Calculate yaw acceleration (rotation)
        const float momentOfInertia = carModel.mass * (carModel.wheelbase * carModel.wheelbase) / 12.0f; // Simplified
        const float yawAcceleration = yawMoment / momentOfInertia;
        
        // Update velocities using forces (semi-implicit Euler integration)
        carModel.velocity += longitudinalAcceleration * deltaTime;
        carModel.lateralVelocity += lateralAcceleration * deltaTime;
        carModel.yawRate += yawAcceleration * deltaTime;
        
        // Add extra turning force at low speeds to improve maneuverability
        if (fabsf(carModel.velocity) > 0.1f) {
            // Low-speed steering boost - higher effect at lower speeds
            const float steeringBoostFactor = fmax(0.0f, 1.0f - (fabsf(carModel.velocity) / 5.0f));
            if (steeringBoostFactor > 0.0f) {
                // Apply boost only when car is moving and steering is applied
                carModel.yawRate += effectiveSteeringAngle * steeringBoostFactor * 1.5f * deltaTime * 
                                  (carModel.velocity >= 0.0f ? 1.0f : -1.0f);
            }
        }
        
        // Apply velocity damping to simulate tire slip and natural stabilization
        carModel.lateralVelocity *= 0.95f; // Lateral slip damping
        carModel.yawRate *= 0.95f; // Yaw damping (simulates stabilizing forces)
        
        // Update wheel angular velocity based on engine and braking
        // When braking, wheels slow down faster than the car
        const float wheelTorqueNet = carModel.wheelTorque - (brakeForce * carModel.wheelRadius);
        const float wheelAcceleration = wheelTorqueNet / carModel.wheelMomentOfInertia;
        carModel.wheelAngularVelocity += wheelAcceleration * deltaTime;
        
        // Update position based on velocity
        // Forward is -Z, right is +X in our coordinate system
        carModel.position.z += carModel.velocity * cosf(carModel.heading) * deltaTime;
        carModel.position.x -= carModel.velocity * sinf(carModel.heading) * deltaTime;
        
        // Add lateral velocity component
        carModel.position.z += carModel.lateralVelocity * sinf(carModel.heading) * deltaTime;
        carModel.position.x += carModel.lateralVelocity * cosf(carModel.heading) * deltaTime;
        
        // Update heading based on yaw rate
        carModel.heading += carModel.yawRate * deltaTime;
        
        // Update engine RPM based on wheel speed and gear ratio
        // When clutch is engaged (throttle > 0)
        if (carModel.throttleInput > 0.0f) {
            carModel.engineRPM = (carModel.wheelAngularVelocity * totalGearRatio * 60.0f) / (2.0f * PI);
            carModel.engineRPM = fmax(carModel.idleRPM, carModel.engineRPM);
            carModel.engineRPM = fmin(carModel.maxRPM, carModel.engineRPM);
        } else {
            // When idle or braking, RPM tends toward idle
            carModel.engineRPM += (carModel.idleRPM - carModel.engineRPM) * deltaTime * 2.0f;
        }
        
        // Update wheel rotation for visuals
        wheelRotation += carModel.wheelAngularVelocity * deltaTime;
        
        // Update path trail
        pathTrail.AddPoint(carModel.position);
        
        // Update camera position
        // Calculate ideal camera position based on car's heading
        const float distance = 10.0f; // Distance behind the car
        const float height = 4.0f;    // Height above the car
        const float lookAheadDistance = 3.0f; // Distance to look ahead of the car
        
        // Position camera behind and above car
        Vector3 cameraOffset = {
            sinf(carModel.heading) * distance,
            height,
            -cosf(carModel.heading) * distance
        };
        
        // Calculate look-at position (slightly ahead of the car)
        Vector3 lookAtOffset = {
            -sinf(carModel.heading) * lookAheadDistance,
            0.0f,
            cosf(carModel.heading) * lookAheadDistance
        };
        Vector3 lookAtPosition = Vector3Add(carModel.position, lookAtOffset);
        
        // Calculate camera smoothing factor based on speed
        // Faster speed = quicker camera response
        const float velocityFactor = fmin(fabsf(carModel.velocity) / 5.0f, 1.0f);
        const float smoothFactor = 0.03f + velocityFactor * 0.07f;
        
        // Smoothly transition camera position
        cameraTargetPosition = Vector3Add(carModel.position, cameraOffset);
        camera.position = Vector3Lerp(camera.position, cameraTargetPosition, smoothFactor);
        
        // Smoothly transition look-at target
        cameraLookAt = Vector3Lerp(cameraLookAt, lookAtPosition, smoothFactor);
        camera.target = cameraLookAt;
        
        // Start drawing
        BeginDrawing();
            ClearBackground(SKYBLUE);
            
            BeginMode3D(camera);
                // Draw ground plane
                DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 100.0f, 100.0f }, DARKGREEN);
                
                // Draw grid for better visual reference
                DrawGrid(100, 1.0f);
                
                // Draw path trail
                pathTrail.Draw();
                
                // Set car transform matrix
                Matrix carTransform = MatrixIdentity();
                carTransform = MatrixMultiply(carTransform, MatrixRotateY(-carModel.heading));
                carTransform = MatrixMultiply(carTransform, MatrixTranslate(carModel.position.x, 0.4f, carModel.position.z));
                
                // Draw car body
                DrawModelEx(carBody, Vector3Zero(), (Vector3){ 0.0f, 1.0f, 0.0f }, 0.0f, (Vector3){ 1.0f, 1.0f, 1.0f }, BLUE);
                
                // Draw car hood
                Matrix hoodMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.7f, 1.5f), carTransform);
                DrawMesh(carHood.meshes[0], carHood.materials[0], hoodMatrix);
                
                // Draw car trunk
                Matrix trunkMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.7f, -1.6f), carTransform);
                DrawMesh(carTrunk.meshes[0], carTrunk.materials[0], trunkMatrix);
                
                // Draw car cabin
                Matrix cabinMatrix = MatrixMultiply(MatrixTranslate(0.0f, 1.15f, 0.0f), carTransform);
                DrawMesh(carCabin.meshes[0], carCabin.materials[0], cabinMatrix);
                
                // Draw car roof
                Matrix roofMatrix = MatrixMultiply(MatrixTranslate(0.0f, 1.55f, 0.0f), carTransform);
                DrawMesh(carRoof.meshes[0], carRoof.materials[0], roofMatrix);
                
                // Draw wheels
                // Rear left wheel
                Matrix wheelRLMatrix = MatrixIdentity();
                wheelRLMatrix = MatrixMultiply(wheelRLMatrix, MatrixRotateX(wheelRotation));
                wheelRLMatrix = MatrixMultiply(wheelRLMatrix, MatrixTranslate(-1.05f, 0.4f, -1.5f));
                wheelRLMatrix = MatrixMultiply(wheelRLMatrix, carTransform);
                DrawMesh(wheelModel.meshes[0], wheelModel.materials[0], wheelRLMatrix);
                
                // Rear right wheel
                Matrix wheelRRMatrix = MatrixIdentity();
                wheelRRMatrix = MatrixMultiply(wheelRRMatrix, MatrixRotateX(wheelRotation));
                wheelRRMatrix = MatrixMultiply(wheelRRMatrix, MatrixTranslate(1.05f, 0.4f, -1.5f));
                wheelRRMatrix = MatrixMultiply(wheelRRMatrix, carTransform);
                DrawMesh(wheelModel.meshes[0], wheelModel.materials[0], wheelRRMatrix);
                
                // Front left wheel (with steering)
                Matrix wheelFLMatrix = MatrixIdentity();
                wheelFLMatrix = MatrixMultiply(wheelFLMatrix, MatrixRotateX(wheelRotation));
                wheelFLMatrix = MatrixMultiply(wheelFLMatrix, MatrixRotateY(-carModel.steeringAngle));
                wheelFLMatrix = MatrixMultiply(wheelFLMatrix, MatrixTranslate(-1.05f, 0.4f, 1.5f));
                wheelFLMatrix = MatrixMultiply(wheelFLMatrix, carTransform);
                DrawMesh(wheelModel.meshes[0], wheelModel.materials[0], wheelFLMatrix);
                
                // Front right wheel (with steering)
                Matrix wheelFRMatrix = MatrixIdentity();
                wheelFRMatrix = MatrixMultiply(wheelFRMatrix, MatrixRotateX(wheelRotation));
                wheelFRMatrix = MatrixMultiply(wheelFRMatrix, MatrixRotateY(-carModel.steeringAngle));
                wheelFRMatrix = MatrixMultiply(wheelFRMatrix, MatrixTranslate(1.05f, 0.4f, 1.5f));
                wheelFRMatrix = MatrixMultiply(wheelFRMatrix, carTransform);
                DrawMesh(wheelModel.meshes[0], wheelModel.materials[0], wheelFRMatrix);
                
                // Draw headlights
                Matrix headlightLeftMatrix = MatrixMultiply(MatrixTranslate(-0.6f, 0.5f, 2.1f), carTransform);
                DrawMesh(headlight.meshes[0], headlight.materials[0], headlightLeftMatrix);
                
                Matrix headlightRightMatrix = MatrixMultiply(MatrixTranslate(0.6f, 0.5f, 2.1f), carTransform);
                DrawMesh(headlight.meshes[0], headlight.materials[0], headlightRightMatrix);
                
                // Draw taillights
                Matrix taillightLeftMatrix = MatrixMultiply(MatrixTranslate(-0.7f, 0.6f, -2.1f), carTransform);
                DrawMesh(taillight.meshes[0], taillight.materials[0], taillightLeftMatrix);
                
                Matrix taillightRightMatrix = MatrixMultiply(MatrixTranslate(0.7f, 0.6f, -2.1f), carTransform);
                DrawMesh(taillight.meshes[0], taillight.materials[0], taillightRightMatrix);
                
                // Draw bumpers
                Matrix frontBumperMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.35f, 2.1f), carTransform);
                DrawMesh(bumper.meshes[0], bumper.materials[0], frontBumperMatrix);
                
                Matrix rearBumperMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.35f, -2.1f), carTransform);
                DrawMesh(bumper.meshes[0], bumper.materials[0], rearBumperMatrix);
            EndMode3D();
            
            // Draw UI
            DrawRectangle(10, 10, 300, 220, Fade(BLACK, 0.7f));
            DrawText("Dynamic Engine Car Simulation", 20, 20, 20, WHITE);
            DrawText("Controls:", 20, 50, 10, WHITE);
            DrawText("- Up/Down Arrow or W/S: Throttle/Brake", 20, 70, 10, WHITE);
            DrawText("- Left/Right Arrow or A/D: Steer", 20, 90, 10, WHITE);
            DrawText("- R: Reset position", 20, 110, 10, WHITE);
            
            // Display stats
            char speedText[64];
            sprintf(speedText, "Speed: %.2f m/s", fabsf(carModel.velocity));
            DrawText(speedText, 20, 140, 10, WHITE);
            
            char rpmText[64];
            sprintf(rpmText, "RPM: %d", (int)carModel.engineRPM);
            DrawText(rpmText, 20, 160, 10, WHITE);
            
            DrawText("Gear: 1 (Fixed)", 20, 180, 10, WHITE);
            
            char throttleText[64];
            sprintf(throttleText, "Throttle: %d%%", (int)(carModel.throttleInput * 100.0f));
            DrawText(throttleText, 20, 200, 10, WHITE);
            
            char steeringText[64];
            sprintf(steeringText, "Steering Angle: %.2f°", carModel.steeringAngle * (180.0f / PI));
            DrawText(steeringText, 20, 220, 10, WHITE);
            
            char forceText[64];
            sprintf(forceText, "Traction Force: %d N", (int)carModel.lastAppliedForce);
            DrawText(forceText, 20, 240, 10, WHITE);
            
            char yawText[64];
            sprintf(yawText, "Yaw Rate: %.2f rad/s", carModel.yawRate);
            DrawText(yawText, 20, 260, 10, WHITE);
        EndDrawing();
    }
    
    // De-Initialization
    UnloadModel(carBody);
    UnloadModel(carHood);
    UnloadModel(carTrunk);
    UnloadModel(carCabin);
    UnloadModel(carRoof);
    UnloadModel(wheelModel);
    UnloadModel(headlight);
    UnloadModel(taillight);
    UnloadModel(bumper);
    
    CloseWindow();        // Close window and OpenGL context
    
    return 0;
}