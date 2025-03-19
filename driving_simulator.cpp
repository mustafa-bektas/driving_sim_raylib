#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <ctime>
#include <random>

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

// Enhanced particle system
struct ParticleSystem {
    struct Particle {
        Vector3 position;
        Vector3 velocity;
        Color color;
        float size;
        float life;
        float maxLife;
    };
    
    std::vector<Particle> particles;
    int maxParticles;
    
    ParticleSystem(int max) : maxParticles(max) {
        particles.reserve(maxParticles);
    }
    
    void AddParticle(Vector3 position, Vector3 velocity, Color color, float size, float life) {
        if (particles.size() >= maxParticles) return;
        
        Particle p;
        p.position = position;
        p.velocity = velocity;
        p.color = color;
        p.size = size;
        p.life = life;
        p.maxLife = life;
        
        particles.push_back(p);
    }
    
    void Update(float deltaTime) {
        for (size_t i = 0; i < particles.size(); i++) {
            particles[i].position = Vector3Add(particles[i].position, 
                Vector3Scale(particles[i].velocity, deltaTime));
            particles[i].life -= deltaTime;
            
            // Apply gravity
            particles[i].velocity.y -= 9.8f * deltaTime;
            
            // Fade out as life decreases
            float lifeFactor = particles[i].life / particles[i].maxLife;
            particles[i].color.a = (unsigned char)(255.0f * lifeFactor);
            particles[i].size *= 0.99f;
        }
        
        // Remove dead particles
        particles.erase(
            std::remove_if(particles.begin(), particles.end(), 
                [](const Particle& p) { return p.life <= 0.0f; }),
            particles.end()
        );
    }
    
    void Draw() {
        for (const auto& p : particles) {
            DrawSphere(p.position, p.size, p.color);
        }
    }
};

// Enhanced path trail with dust effect
struct PathTrail {
    std::vector<Vector3> points;
    std::vector<float> widths;
    std::vector<Color> colors;
    int maxPoints = 500;
    
    void AddPoint(Vector3 position, float width, float speed) {
        // Add current position to path
        if (points.empty() || Vector3Distance(points.back(), position) > 0.5f) {
            points.push_back(position);
            
            // Width based on speed for visual effect
            widths.push_back(width * (0.5f + fmin(speed / 20.0f, 1.0f)));
            
            // Color based on speed
            unsigned char intensity = (unsigned char)(fmin(speed * 10.0f, 255.0f));
            colors.push_back((Color){ intensity, intensity, intensity, 128 });
            
            // Limit path length
            if (points.size() > maxPoints) {
                points.erase(points.begin());
                widths.erase(widths.begin());
                colors.erase(colors.begin());
            }
        }
    }
    
    void Draw() {
        for (size_t i = 0; i < points.size() - 1; i++) {
            // Create a rectangle between points for better visuals
            Vector3 direction = Vector3Normalize(
                Vector3Subtract(points[i + 1], points[i])
            );
            
            // Calculate perpendicular vector
            Vector3 perpendicular = {
                -direction.z,
                0.0f,
                direction.x
            };
            
            // Draw as a quad between points
            Vector3 p1 = Vector3Add(points[i], Vector3Scale(perpendicular, widths[i] / 2.0f));
            Vector3 p2 = Vector3Subtract(points[i], Vector3Scale(perpendicular, widths[i] / 2.0f));
            Vector3 p3 = Vector3Subtract(points[i + 1], Vector3Scale(perpendicular, widths[i + 1] / 2.0f));
            Vector3 p4 = Vector3Add(points[i + 1], Vector3Scale(perpendicular, widths[i + 1] / 2.0f));
            
            // Draw quad using triangles
            DrawTriangle3D(p1, p2, p3, colors[i]);
            DrawTriangle3D(p1, p3, p4, colors[i]);
        }
    }
};

// Environment object for decoration
struct EnvironmentObject {
    Model model;
    Vector3 position;
    Vector3 scale;
    float rotation;
    bool collisionEnabled;
    BoundingBox bounds;
    
    EnvironmentObject(Model m, Vector3 pos, Vector3 sc, float rot, bool collision) :
        model(m), position(pos), scale(sc), rotation(rot), collisionEnabled(collision) {
        
        // Get model bounding box using Raylib function
        BoundingBox modelBox = GetModelBoundingBox(model);
        
        // Scale the bounding box
        bounds.min = Vector3Scale(modelBox.min, sc.x);
        bounds.max = Vector3Scale(modelBox.max, sc.x);
        
        // Transform bounding box to world space
        Matrix transform = MatrixIdentity();
        transform = MatrixMultiply(transform, MatrixScale(sc.x, sc.y, sc.z));
        transform = MatrixMultiply(transform, MatrixRotateY(rot));
        transform = MatrixMultiply(transform, MatrixTranslate(pos.x, pos.y, pos.z));
        
        bounds = GetBoundingBoxTransform(bounds, transform);
    }
    
    void Draw() {
        DrawModelEx(model, position, (Vector3){ 0.0f, 1.0f, 0.0f }, 
                    rotation, scale, WHITE);
    }
    
    // Helper for transforming bounding box
    BoundingBox GetBoundingBoxTransform(BoundingBox box, Matrix transform) {
        // Get the 8 corners of the bounding box
        Vector3 vertices[8] = {
            { box.min.x, box.min.y, box.min.z },
            { box.max.x, box.min.y, box.min.z },
            { box.min.x, box.max.y, box.min.z },
            { box.max.x, box.max.y, box.min.z },
            { box.min.x, box.min.y, box.max.z },
            { box.max.x, box.min.y, box.max.z },
            { box.min.x, box.max.y, box.max.z },
            { box.max.x, box.max.y, box.max.z }
        };
        
        // Transform all vertices
        Vector3 transformedVertices[8];
        for (int i = 0; i < 8; i++) {
            transformedVertices[i] = Vector3Transform(vertices[i], transform);
        }
        
        // Find new min and max
        Vector3 min = transformedVertices[0];
        Vector3 max = transformedVertices[0];
        
        for (int i = 1; i < 8; i++) {
            min.x = fmin(min.x, transformedVertices[i].x);
            min.y = fmin(min.y, transformedVertices[i].y);
            min.z = fmin(min.z, transformedVertices[i].z);
            
            max.x = fmax(max.x, transformedVertices[i].x);
            max.y = fmax(max.y, transformedVertices[i].y);
            max.z = fmax(max.z, transformedVertices[i].z);
        }
        
        return (BoundingBox){ min, max };
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
    bool c = false;  // Camera toggle
    bool h = false;  // Headlight toggle
    
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
        c = IsKeyPressed(KEY_C);
        h = IsKeyPressed(KEY_H);
    }
};

// Function to create a detailed wheel mesh
Model CreateDetailedWheelModel(Texture2D tireTex, Texture2D rimTex) {
    // Create a more detailed wheel with separate rim and tire meshes
    // Tire is a cylinder with a larger radius
    Mesh tireMesh = GenMeshCylinder(0.4f, 0.25f, 16);
    
    // Generate UVs for tire texture wrapping
    for (int i = 0; i < tireMesh.vertexCount; i++) {
        float *texCoords = &tireMesh.texcoords[i*2];
        float *vertices = &tireMesh.vertices[i*3];
        
        // Calculate UV coordinates based on position
        float u = atan2f(vertices[0], vertices[2]) / (2 * PI) + 0.5f;
        float v = vertices[1] / 0.25f + 0.5f;
        
        texCoords[0] = u;
        texCoords[1] = v;
    }
    
    Model tireModel = LoadModelFromMesh(tireMesh);
    tireModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = tireTex;
    
    // Create rim (inner cylinder with smaller radius)
    Mesh rimMesh = GenMeshCylinder(0.3f, 0.25f, 12);
    
    // Generate UVs for rim texture wrapping
    for (int i = 0; i < rimMesh.vertexCount; i++) {
        float *texCoords = &rimMesh.texcoords[i*2];
        float *vertices = &rimMesh.vertices[i*3];
        
        // Calculate UV coordinates based on position
        float u = atan2f(vertices[0], vertices[2]) / (2 * PI) + 0.5f;
        float v = vertices[1] / 0.25f + 0.5f;
        
        texCoords[0] = u;
        texCoords[1] = v;
    }
    
    Model rimModel = LoadModelFromMesh(rimMesh);
    rimModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = rimTex;
    
    // Rotate wheels to match orientation
    tireModel.transform = MatrixRotateZ(PI / 2.0f);
    rimModel.transform = MatrixRotateZ(PI / 2.0f);
    
    // For simplicity, we'll just return the tire model
    // In a more advanced version, you might want to combine the models
    return tireModel;
}

// Function to create a detailed car model
void CreateDetailedCarModel(Model &body, Model &hood, Model &trunk, 
                        Model &cabin, Model &roof, Model &bumperFront, 
                        Model &bumperRear, Texture2D carTex, Texture2D glassTex) {
    // Create more detailed car parts with proper meshes
    
    // Car body - create with appropriate dimensions
    body = LoadModelFromMesh(GenMeshCube(2.0f, 0.4f, 4.2f));
    
    // Set car texture
    body.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = carTex;
    
    // Create other components
    hood = LoadModelFromMesh(GenMeshCube(1.9f, 0.2f, 1.0f));
    hood.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = carTex;
    
    trunk = LoadModelFromMesh(GenMeshCube(1.9f, 0.25f, 0.8f));
    trunk.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = carTex;
    
    cabin = LoadModelFromMesh(GenMeshCube(1.8f, 0.7f, 2.2f));
    cabin.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = glassTex;
    
    roof = LoadModelFromMesh(GenMeshCube(1.7f, 0.15f, 2.0f));
    roof.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = carTex;
    
    // Bumpers
    bumperFront = LoadModelFromMesh(GenMeshCube(2.0f, 0.3f, 0.2f));
    bumperFront.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = carTex;
    
    bumperRear = LoadModelFromMesh(GenMeshCube(2.0f, 0.3f, 0.2f));
    bumperRear.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = carTex;
    
    // Add material properties to all parts
    Material carMaterial = body.materials[0];
    carMaterial.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    carMaterial.maps[MATERIAL_MAP_SPECULAR].color = (Color){ 255, 255, 255, 128 };
    
    // Apply material to all parts
    body.materials[0] = carMaterial;
    hood.materials[0] = carMaterial;
    trunk.materials[0] = carMaterial;
    roof.materials[0] = carMaterial;
    bumperFront.materials[0] = carMaterial;
    bumperRear.materials[0] = carMaterial;
    
    // Special material for cabin (glass effect)
    Material glassMaterial = cabin.materials[0];
    glassMaterial.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 255, 255, 255, 200 };
    glassMaterial.maps[MATERIAL_MAP_SPECULAR].color = (Color){ 255, 255, 255, 255 };
    cabin.materials[0] = glassMaterial;
}

// Function to create environment objects
std::vector<EnvironmentObject> CreateEnvironment(Texture2D grassTex, Texture2D roadTex) {
    std::vector<EnvironmentObject> objects;
    
    // Create trees, rocks, buildings, etc.
    
    // Random number generator
    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));
    std::uniform_real_distribution<float> distPos(-40.0f, 40.0f);
    std::uniform_real_distribution<float> distScale(0.7f, 1.3f);
    std::uniform_real_distribution<float> distRot(0.0f, 2.0f * PI);
    
    // Add trees
    Model treeModel = LoadModelFromMesh(GenMeshCylinder(0.3f, 3.0f, 8));
    Model leafModel = LoadModelFromMesh(GenMeshCone(2.0f, 3.0f, 8));
    
    for (int i = 0; i < 20; i++) {
        float x = distPos(rng);
        float z = distPos(rng);
        
        // Keep trees away from the start position
        if (Vector3Distance((Vector3){x, 0, z}, (Vector3){0, 0, 0}) < 15.0f)
            continue;
        
        float scale = distScale(rng);
        float rotation = distRot(rng);
        
        // Add tree trunk
        objects.push_back(EnvironmentObject(
            treeModel, 
            (Vector3){x, 0.0f, z}, 
            (Vector3){scale, scale, scale}, 
            rotation, 
            true
        ));
        
        // Add tree leaves on top
        objects.push_back(EnvironmentObject(
            leafModel, 
            (Vector3){x, 3.0f * scale, z}, 
            (Vector3){scale, scale, scale}, 
            rotation, 
            false
        ));
    }
    
    // Add rocks
    Model rockModel = LoadModelFromMesh(GenMeshSphere(1.0f, 8, 8));
    
    for (int i = 0; i < 15; i++) {
        float x = distPos(rng);
        float z = distPos(rng);
        
        // Keep rocks away from the start position
        if (Vector3Distance((Vector3){x, 0, z}, (Vector3){0, 0, 0}) < 15.0f)
            continue;
        
        float scale = distScale(rng) * 0.5f;
        float rotation = distRot(rng);
        
        objects.push_back(EnvironmentObject(
            rockModel, 
            (Vector3){x, 0.0f, z}, 
            (Vector3){scale * 1.5f, scale, scale * 1.2f}, 
            rotation, 
            true
        ));
    }
    
    // Create a simple building
    Model buildingModel = LoadModelFromMesh(GenMeshCube(5.0f, 8.0f, 5.0f));
    
    objects.push_back(EnvironmentObject(
        buildingModel, 
        (Vector3){20.0f, 4.0f, 20.0f}, 
        (Vector3){1.0f, 1.0f, 1.0f}, 
        PI/4.0f, 
        true
    ));
    
    objects.push_back(EnvironmentObject(
        buildingModel, 
        (Vector3){-25.0f, 4.0f, -25.0f}, 
        (Vector3){1.2f, 0.8f, 1.0f}, 
        PI/6.0f, 
        true
    ));
    
    // Create track/road
    Model roadSegment = LoadModelFromMesh(GenMeshCube(5.0f, 0.1f, 5.0f));
    roadSegment.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = roadTex;
    
    // Create a circular track
    const int numSegments = 24;
    const float radius = 25.0f;
    
    for (int i = 0; i < numSegments; i++) {
        float angle = ((float)i / numSegments) * 2.0f * PI;
        float x = cosf(angle) * radius;
        float z = sinf(angle) * radius;
        
        objects.push_back(EnvironmentObject(
            roadSegment, 
            (Vector3){x, 0.01f, z}, 
            (Vector3){1.0f, 1.0f, 1.0f}, 
            angle, 
            false
        ));
    }
    
    // Set texture for all objects
    for (auto& obj : objects) {
        // Set tree texture for trunks
        if (obj.model.meshes[0].vertexCount == treeModel.meshes[0].vertexCount) {
            obj.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = BROWN;
        }
        // Set leaf texture
        else if (obj.model.meshes[0].vertexCount == leafModel.meshes[0].vertexCount) {
            obj.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GREEN;
        }
        // Set rock texture
        else if (obj.model.meshes[0].vertexCount == rockModel.meshes[0].vertexCount) {
            obj.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GRAY;
        }
        // Set building texture
        else if (obj.model.meshes[0].vertexCount == buildingModel.meshes[0].vertexCount) {
            obj.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = LIGHTGRAY;
        }
    }
    
    return objects;
}

// Create dynamic sky and lighting
struct SkySystem {
    Texture2D skyTexture;
    Color skyColor;
    Color groundColor;
    float sunPosition;
    Color sunColor;
    
    void Update(float deltaTime) {
        // Slowly move sun position for day/night cycle
        sunPosition += deltaTime * 0.01f;
        if (sunPosition > 2.0f * PI) sunPosition -= 2.0f * PI;
        
        // Change sky color based on sun position
        float dayFactor = sinf(sunPosition) * 0.5f + 0.5f;
        
        // Blend between day and night colors
        skyColor.r = (unsigned char)(50 + 175 * dayFactor);
        skyColor.g = (unsigned char)(120 + 135 * dayFactor);
        skyColor.b = (unsigned char)(180 + 75 * dayFactor);
        
        // Sun color shifts from yellow during day to red at sunset
        float sunsetFactor = powf(1.0f - fabsf(sinf(sunPosition)), 4.0f);
        sunColor.r = 255;
        sunColor.g = (unsigned char)(200.0f * (1.0f - sunsetFactor * 0.7f));
        sunColor.b = (unsigned char)(100.0f * (1.0f - sunsetFactor * 0.9f));
        
        // Ground color is darker version of sky
        groundColor.r = skyColor.r / 2;
        groundColor.g = skyColor.g / 2;
        groundColor.b = skyColor.b / 2;
    }
    
    void Draw() {
        // Draw sky as a gradient
        DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight()/2, skyColor, groundColor);
        
        // Draw gradient for ground
        DrawRectangleGradientV(0, GetScreenHeight()/2, GetScreenWidth(), GetScreenHeight()/2, groundColor, (Color){20, 20, 20, 255});
        
        // Draw sun
        float sunX = (sinf(sunPosition) * 0.5f + 0.5f) * GetScreenWidth();
        float sunY = (cosf(sunPosition) * 0.5f + 0.1f) * GetScreenHeight();
        
        // Only draw sun when above horizon
        if (cosf(sunPosition) > -0.1f) {
            DrawCircleGradient(sunX, sunY, 40.0f, sunColor, BLANK);
        }
    }
};

// Create shadow system
struct ShadowSystem {
    RenderTexture2D shadowMap;
    Camera shadowCamera;
    float lightDir[3] = {-0.5f, -0.5f, -0.5f};
    
    ShadowSystem(int resolution = 1024) {
        shadowMap = LoadRenderTexture(resolution, resolution);
        shadowCamera = {};
        shadowCamera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
        shadowCamera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
        shadowCamera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
        shadowCamera.fovy = 45.0f;
        shadowCamera.projection = CAMERA_ORTHOGRAPHIC;
    }
    
    void UpdateLightDir(float sunPosition) {
        lightDir[0] = -sinf(sunPosition);
        lightDir[1] = -cosf(sunPosition);
        lightDir[2] = 0.3f;
        
        // Update shadow camera position based on light direction
        shadowCamera.position.x = -lightDir[0] * 20.0f;
        shadowCamera.position.y = -lightDir[1] * 20.0f;
        shadowCamera.position.z = -lightDir[2] * 20.0f;
    }
    
    void BeginShadowMode() {
        BeginTextureMode(shadowMap);
            ClearBackground(WHITE);
            BeginMode3D(shadowCamera);
    }
    
    void EndShadowMode() {
            EndMode3D();
        EndTextureMode();
    }
    
    // Apply shadows as a simple projection (simplified approach)
    void ApplyShadows(Vector3 position, float radius) {
        // Project position to ground
        Vector3 shadowPos = position;
        shadowPos.y = 0.01f;
        
        // Calculate fade based on height
        float alpha = 1.0f - position.y / 5.0f;
        alpha = fmax(0.0f, fmin(0.5f, alpha));
        
        // Draw shadow as a simple circle
        DrawCircle3D(shadowPos, radius, (Vector3){1,0,0}, 90.0f, (Color){0, 0, 0, (unsigned char)(alpha*255)});
    }
};

// Create reflection system
struct ReflectionSystem {
    RenderTexture2D reflectionMap;
    
    ReflectionSystem(int width, int height) {
        reflectionMap = LoadRenderTexture(width, height);
    }
    
    void BeginReflectionMode(Camera camera, float waterHeight) {
        // Create a flipped camera for reflections
        Camera reflectionCamera = camera;
        reflectionCamera.position.y = 2.0f * waterHeight - camera.position.y;
        reflectionCamera.target.y = 2.0f * waterHeight - camera.target.y;
        reflectionCamera.up.y = -camera.up.y;
        
        BeginTextureMode(reflectionMap);
            ClearBackground(SKYBLUE);
            BeginMode3D(reflectionCamera);
    }
    
    void EndReflectionMode() {
            EndMode3D();
        EndTextureMode();
    }
    
    void DrawReflectionSurface(float width, float length, float height, Texture2D waterTex) {
        // Draw a semi-transparent water surface with reflection
        Mesh waterMesh = GenMeshPlane(width, length, 10, 10);
        Model waterModel = LoadModelFromMesh(waterMesh);
        
        // Apply water texture
        waterModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = waterTex;
        
        // Make water semi-transparent
        waterModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 255, 255, 255, 200 };
        
        // Draw water surface
        DrawModelEx(waterModel, (Vector3){0, height, 0}, (Vector3){0, 1, 0}, 0.0f, (Vector3){1, 1, 1}, WHITE);
        
        // Clean up
        UnloadModel(waterModel);
    }
};

int main() {
    // Enhanced initialization with higher resolution
    const int screenWidth = 1280;
    const int screenHeight = 720;
    
    // Enable MSAA for better antialiasing
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screenWidth, screenHeight, "Visually Enhanced Car Simulator");
    
    // Create procedural textures directly (no file loading)
    // Create tire texture (black with tread pattern)
    Image tireTreadImg = GenImageChecked(256, 256, 32, 32, BLACK, DARKGRAY);
    Texture2D tireTex = LoadTextureFromImage(tireTreadImg);
    UnloadImage(tireTreadImg);
    
    // Create rim texture (metallic looking)
    Image rimImg = GenImageChecked(256, 256, 16, 16, LIGHTGRAY, WHITE);
    Texture2D rimTex = LoadTextureFromImage(rimImg);
    UnloadImage(rimImg);
    
    // Create car texture (blue with subtle pattern)
    Image carImg = GenImageColor(256, 256, BLUE);
    // Add some variation to the car texture
    for (int y = 0; y < 256; y += 32) {
        for (int x = 0; x < 256; x += 32) {
            ImageDrawRectangle(&carImg, x, y, 30, 30, (Color){70, 70, 220, 255});
        }
    }
    Texture2D carTex = LoadTextureFromImage(carImg);
    UnloadImage(carImg);
    
    // Create glass texture (semi-transparent blue)
    Image glassImg = GenImageColor(256, 256, (Color){100, 200, 255, 180});
    Texture2D glassTex = LoadTextureFromImage(glassImg);
    UnloadImage(glassImg);
    
    // Create road texture (asphalt-like)
    Image roadImg = GenImageChecked(256, 256, 8, 8, DARKGRAY, GRAY);
    // Add some noise to make it look more like asphalt
    for (int i = 0; i < 5000; i++) {
        int x = GetRandomValue(0, 255);
        int y = GetRandomValue(0, 255);
        Color pixel = {
            (unsigned char)GetRandomValue(50, 70),
            (unsigned char)GetRandomValue(50, 70),
            (unsigned char)GetRandomValue(50, 70),
            255
        };
        ImageDrawPixel(&roadImg, x, y, pixel);
    }
    Texture2D roadTex = LoadTextureFromImage(roadImg);
    UnloadImage(roadImg);
    
    // Create grass texture (green with variations)
    Image grassImg = GenImageColor(256, 256, DARKGREEN);
    // Add some variation to make it look more like grass
    for (int i = 0; i < 10000; i++) {
        int x = GetRandomValue(0, 255);
        int y = GetRandomValue(0, 255);
        Color pixel = {
            (unsigned char)GetRandomValue(0, 50),
            (unsigned char)GetRandomValue(100, 150),
            (unsigned char)GetRandomValue(0, 50),
            255
        };
        ImageDrawPixel(&grassImg, x, y, pixel);
    }
    Texture2D grassTex = LoadTextureFromImage(grassImg);
    UnloadImage(grassImg);
    
    // Create water texture (blue with wave-like pattern)
    Image waterImg = GenImageColor(256, 256, (Color){80, 120, 255, 200});
    // Add wave-like pattern
    for (int y = 0; y < 256; y++) {
        for (int x = 0; x < 256; x++) {
            float noise = sinf(x * 0.05f) * 10.0f + sinf(y * 0.03f) * 10.0f;
            Color pixel = {
                (unsigned char)fmin(255, fmax(0, 80 + noise)),
                (unsigned char)fmin(255, fmax(0, 120 + noise)),
                (unsigned char)fmin(255, fmax(0, 255 + noise/2)),
                200
            };
            ImageDrawPixel(&waterImg, x, y, pixel);
        }
    }
    Texture2D waterTex = LoadTextureFromImage(waterImg);
    
    // Set our camera
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 7.0f, 15.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    
    // Also create a first-person camera for switching views
    Camera fpCamera = { 0 };
    fpCamera.position = (Vector3){ 0.0f, 1.8f, 0.0f }; // Initially positioned at origin
    fpCamera.target = (Vector3){ 0.0f, 1.8f, 1.0f };   // Looking forward
    fpCamera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    fpCamera.fovy = 75.0f;
    fpCamera.projection = CAMERA_PERSPECTIVE;
    
    // Camera smooth parameters
    Vector3 cameraTargetPosition = { 0.0f, 0.0f, 0.0f };
    
    // Vector3 to keep track of camera look-at position for smooth transitions
    Vector3 cameraLookAt = { 0.0f, 0.0f, 0.0f };
    
    // Create sky system
    SkySystem skySystem;
    skySystem.sunPosition = PI / 4.0f; // Start at morning
    
    // Create shadow system
    ShadowSystem shadowSystem(2048); // Higher resolution for better quality
    
    // Create particle system for effects
    ParticleSystem exhaustParticles(300);
    ParticleSystem skidParticles(500);
    
    // Car physics model
    CarModel carModel;
    
    // Create visual models for car parts
    Model carBody, carHood, carTrunk, carCabin, carRoof, carBumperFront, carBumperRear;
    CreateDetailedCarModel(carBody, carHood, carTrunk, carCabin, carRoof, 
                           carBumperFront, carBumperRear, carTex, glassTex);
    
    // Create wheels with better visuals
    Model wheelModel = CreateDetailedWheelModel(tireTex, rimTex);
    
    // Create headlights with proper materials
    Model headlight = LoadModelFromMesh(GenMeshSphere(0.15f, 16, 16));
    
    // Set material for headlights (glowing effect)
    Material headlightMaterial = headlight.materials[0];
    headlightMaterial.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    headlightMaterial.maps[MATERIAL_MAP_EMISSION].color = WHITE; // Emit light
    headlight.materials[0] = headlightMaterial;
    
    // Create taillights with proper materials
    Model taillight = LoadModelFromMesh(GenMeshCube(0.3f, 0.1f, 0.05f));
    
    // Set material for taillights
    Material taillightMaterial = taillight.materials[0];
    taillightMaterial.maps[MATERIAL_MAP_DIFFUSE].color = RED;
    taillightMaterial.maps[MATERIAL_MAP_EMISSION].color = RED; // Emit light
    taillight.materials[0] = taillightMaterial;
    
    // Create enhanced path trail
    PathTrail pathTrail;
    
    // Create environment objects
    std::vector<EnvironmentObject> environmentObjects = CreateEnvironment(grassTex, roadTex);
    
    // Keyboard state
    KeyboardState keys;
    
    // Camera mode flag
    bool firstPersonView = false;
    
    // Headlight flag
    bool headlightsOn = false;
    
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
        pathTrail.widths.clear();
        pathTrail.colors.clear();
    };
    
    // Timing variables
    double previousTime = GetTime();
    float wheelRotation = 0.0f;
    
    // Create shader for materials - but don't try to load from files
    Shader phongShader = { 0 };  // Empty shader, will use Raylib's default
    bool useCustomShader = false;
    
    // Apply shader to car models if available
    if (useCustomShader) {
        carBody.materials[0].shader = phongShader;
        carHood.materials[0].shader = phongShader;
        carTrunk.materials[0].shader = phongShader;
        carRoof.materials[0].shader = phongShader;
        carBumperFront.materials[0].shader = phongShader;
        carBumperRear.materials[0].shader = phongShader;
        wheelModel.materials[0].shader = phongShader;
    }
    
    // Set up materials for lighting
    for (auto& obj : environmentObjects) {
        if (useCustomShader) {
            obj.model.materials[0].shader = phongShader;
        }
    }
    
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
        
        // Handle camera toggle
        if (keys.c) {
            firstPersonView = !firstPersonView;
        }
        
        // Handle headlight toggle
        if (keys.h) {
            headlightsOn = !headlightsOn;
        }
        
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
        
        // Update path trail with speed-dependent width
        pathTrail.AddPoint(carModel.position, 0.2f, fabsf(carModel.velocity));
        
        // Update particle effects
        // Add exhaust particles
        if (carModel.throttleInput > 0.1f) {
            // Calculate exhaust position (at rear of car)
            Vector3 exhaustPosition = {
                carModel.position.x + sinf(carModel.heading) * 2.0f,
                carModel.position.y + 0.3f,
                carModel.position.z - cosf(carModel.heading) * 2.0f
            };
            
            // Add particles with random variation
            for (int i = 0; i < (int)(carModel.throttleInput * 10.0f); i++) {
                Vector3 particleVel = {
                    (GetRandomValue(-10, 10) / 100.0f) + sinf(carModel.heading) * 0.5f,
                    0.5f + (GetRandomValue(0, 100) / 200.0f),
                    (GetRandomValue(-10, 10) / 100.0f) - cosf(carModel.heading) * 0.5f
                };
                
                // Color based on throttle and engine load
                Color particleColor = {
                    (unsigned char)(100 + carModel.throttleInput * 30.0f),
                    (unsigned char)(100 + carModel.throttleInput * 20.0f),
                    (unsigned char)(100),
                    (unsigned char)(150.0f + carModel.throttleInput * 100.0f)
                };
                
                exhaustParticles.AddParticle(
                    exhaustPosition,
                    particleVel,
                    particleColor,
                    0.1f + carModel.throttleInput * 0.1f,
                    0.5f + carModel.throttleInput * 0.5f
                );
            }
        }
        
        // Add skid marks/particles
        const float lateralSlip = fabsf(carModel.lateralVelocity);
        if (lateralSlip > 1.0f || (fabsf(carModel.brakeInput) > 0.8f && fabsf(carModel.velocity) > 5.0f)) {
            // Calculate wheel positions
            float wheelOffset = 0.9f;
            
            Vector3 rearLeftPos = {
                carModel.position.x + cosf(carModel.heading) * wheelOffset,
                carModel.position.y + 0.1f,
                carModel.position.z + sinf(carModel.heading) * wheelOffset
            };
            
            Vector3 rearRightPos = {
                carModel.position.x - cosf(carModel.heading) * wheelOffset,
                carModel.position.y + 0.1f,
                carModel.position.z - sinf(carModel.heading) * wheelOffset
            };
            
            // Add skid particles
            for (int i = 0; i < (int)(lateralSlip * 2.0f); i++) {
                Vector3 particleVel = {
                    (GetRandomValue(-20, 20) / 100.0f),
                    0.1f + (GetRandomValue(0, 50) / 100.0f),
                    (GetRandomValue(-20, 20) / 100.0f)
                };
                
                skidParticles.AddParticle(
                    rearLeftPos,
                    particleVel,
                    (Color){30, 30, 30, 150},
                    0.05f + lateralSlip * 0.02f,
                    0.2f + lateralSlip * 0.1f
                );
                
                skidParticles.AddParticle(
                    rearRightPos,
                    particleVel,
                    (Color){30, 30, 30, 150},
                    0.05f + lateralSlip * 0.02f,
                    0.2f + lateralSlip * 0.1f
                );
            }
        }
        
        // Update particles
        exhaustParticles.Update(deltaTime);
        skidParticles.Update(deltaTime);
        
        // Update the sky and time of day
        skySystem.Update(deltaTime);
        
        // Update shadow light direction based on sun position
        shadowSystem.UpdateLightDir(skySystem.sunPosition);
        
        // Update third-person camera position
        if (!firstPersonView) {
            // Calculate ideal camera position based on car's heading
            const float distance = 8.0f; // Distance behind the car
            const float height = 3.0f;    // Height above the car
            const float lookAheadDistance = 4.0f; // Distance to look ahead of the car
            
            // Position camera behind and above car
            Vector3 cameraOffset = {
                sinf(carModel.heading) * distance,
                height,
                -cosf(carModel.heading) * distance
            };
            
            // Calculate look-at position (slightly ahead of the car)
            Vector3 lookAtOffset = {
                -sinf(carModel.heading) * lookAheadDistance,
                0.5f, // Look slightly above ground
                cosf(carModel.heading) * lookAheadDistance
            };
            Vector3 lookAtPosition = Vector3Add(carModel.position, lookAtOffset);
            
            // Calculate camera smoothing factor based on speed
            // Faster speed = quicker camera response
            const float velocityFactor = fmin(fabsf(carModel.velocity) / 10.0f, 1.0f);
            const float smoothFactor = 0.03f + velocityFactor * 0.1f;
            
            // Smoothly transition camera position
            cameraTargetPosition = Vector3Add(carModel.position, cameraOffset);
            camera.position = Vector3Lerp(camera.position, cameraTargetPosition, smoothFactor);
            
            // Smoothly transition look-at target
            cameraLookAt = Vector3Lerp(cameraLookAt, lookAtPosition, smoothFactor);
            camera.target = cameraLookAt;
        } else {
            // First-person camera position and orientation
            // Position inside car at driver's head position
            Vector3 driverHeadPos = {
                carModel.position.x - sinf(carModel.heading) * 0.5f,
                carModel.position.y + 1.5f, // Driver's eye height
                carModel.position.z + cosf(carModel.heading) * 0.5f
            };
            
            // Look-at position is ahead of the car
            Vector3 lookDir = {
                -sinf(carModel.heading),
                0.0f,
                cosf(carModel.heading)
            };
            
            // Position and target for first-person view
            fpCamera.position = driverHeadPos;
            fpCamera.target = Vector3Add(driverHeadPos, Vector3Scale(lookDir, 1.0f));
        }
        
        // Start drawing
        BeginDrawing();
            ClearBackground(RAYWHITE);
            
            // Draw sky background
            skySystem.Draw();
            
            // Use appropriate camera
            Camera* currentCamera = firstPersonView ? &fpCamera : &camera;
            
            // Begin 3D mode with selected camera
            BeginMode3D(*currentCamera);
                // Draw ground
                DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 100.0f, 100.0f }, (Color){ 
                    50, 120, 50, 255 
                });
                
                // Draw environment objects
                for (auto& obj : environmentObjects) {
                    obj.Draw();
                }
                
                // Draw path trail
                pathTrail.Draw();
                
                // Draw particles
                exhaustParticles.Draw();
                skidParticles.Draw();
                
                // Apply shadows
                shadowSystem.ApplyShadows(carModel.position, 2.0f);
                
                // Set car transform matrix
                Matrix carTransform = MatrixIdentity();
                carTransform = MatrixMultiply(carTransform, MatrixRotateY(-carModel.heading));
                carTransform = MatrixMultiply(carTransform, MatrixTranslate(carModel.position.x, 0.4f, carModel.position.z));
                
                // Draw car body
                DrawModelEx(carBody, Vector3Zero(), (Vector3){ 0.0f, 1.0f, 0.0f }, 0.0f, (Vector3){ 1.0f, 1.0f, 1.0f }, WHITE);
                
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
                
                // Draw wheels with rotation
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
                
                // Draw headlights with glow effect if on
                Color headlightColor = headlightsOn ? 
                    (Color){ 255, 255, 220, 255 } : 
                    (Color){ 200, 200, 200, 255 };
                
                Matrix headlightLeftMatrix = MatrixMultiply(MatrixTranslate(-0.6f, 0.5f, 2.1f), carTransform);
                DrawMesh(headlight.meshes[0], headlight.materials[0], headlightLeftMatrix);
                
                Matrix headlightRightMatrix = MatrixMultiply(MatrixTranslate(0.6f, 0.5f, 2.1f), carTransform);
                DrawMesh(headlight.meshes[0], headlight.materials[0], headlightRightMatrix);
                
                // Draw light beams from headlights if they're on
                if (headlightsOn) {
                    // Get headlight positions in world space
                    Vector3 leftHeadlightPos = {
                        headlightLeftMatrix.m12, headlightLeftMatrix.m13, headlightLeftMatrix.m14
                    };
                    Vector3 rightHeadlightPos = {
                        headlightRightMatrix.m12, headlightRightMatrix.m13, headlightRightMatrix.m14
                    };
                    
                    // Get forward direction vector
                    Vector3 forwardDir = {
                        -sinf(carModel.heading),
                        0.0f,
                        cosf(carModel.heading)
                    };
                    
                    // Draw light beams as cones
                    Vector3 endPosLeft = Vector3Add(leftHeadlightPos, Vector3Scale(forwardDir, 15.0f));
                    Vector3 endPosRight = Vector3Add(rightHeadlightPos, Vector3Scale(forwardDir, 15.0f));
                    
                    // Draw light beams as series of increasingly transparent/wider cylinders
                    for (int i = 0; i < 5; i++) {
                        float t = i / 4.0f; // 0 to 1
                        float distance = 2.0f + t * 13.0f;
                        float radius = 0.2f + t * 1.8f;
                        unsigned char alpha = (unsigned char)(150 * (1.0f - t));
                        
                        // Left headlight beam segment
                        Vector3 segPosLeft = Vector3Add(leftHeadlightPos, Vector3Scale(forwardDir, distance));
                        DrawSphere(segPosLeft, radius, (Color){255, 255, 200, alpha});
                        
                        // Right headlight beam segment
                        Vector3 segPosRight = Vector3Add(rightHeadlightPos, Vector3Scale(forwardDir, distance));
                        DrawSphere(segPosRight, radius, (Color){255, 255, 200, alpha});
                    }
                }
                
                // Draw taillights with brake effect
                Color taillightColor = carModel.brakeInput > 0.0f ? 
                    (Color){ 255, 50, 50, 255 } : 
                    (Color){ 200, 30, 30, 255 };
                
                Matrix taillightLeftMatrix = MatrixMultiply(MatrixTranslate(-0.7f, 0.6f, -2.1f), carTransform);
                taillight.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = taillightColor;
                taillight.materials[0].maps[MATERIAL_MAP_EMISSION].color = taillightColor;
                DrawMesh(taillight.meshes[0], taillight.materials[0], taillightLeftMatrix);
                
                Matrix taillightRightMatrix = MatrixMultiply(MatrixTranslate(0.7f, 0.6f, -2.1f), carTransform);
                DrawMesh(taillight.meshes[0], taillight.materials[0], taillightRightMatrix);
                
                // Draw bumpers
                Matrix frontBumperMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.35f, 2.1f), carTransform);
                DrawMesh(carBumperFront.meshes[0], carBumperFront.materials[0], frontBumperMatrix);
                
                Matrix rearBumperMatrix = MatrixMultiply(MatrixTranslate(0.0f, 0.35f, -2.1f), carTransform);
                DrawMesh(carBumperRear.meshes[0], carBumperRear.materials[0], rearBumperMatrix);
            EndMode3D();
            
            // Draw UI with stylish elements
            // Background panel
            DrawRectangleGradientV(10, 10, 320, 250, 
                (Color){20, 20, 30, 230}, (Color){40, 40, 60, 200});
            DrawRectangleLines(10, 10, 320, 250, (Color){100, 100, 255, 100});
            
            // Title
            DrawText("Enhanced Car Dynamics Simulator", 20, 20, 20, (Color){220, 220, 255, 255});
            DrawLine(20, 45, 310, 45, (Color){100, 100, 255, 100});
            
            // Controls
            DrawText("Controls:", 20, 55, 12, (Color){180, 180, 255, 255});
            DrawText("- Up/Down Arrow or W/S: Throttle/Brake", 30, 75, 10, (Color){200, 200, 255, 255});
            DrawText("- Left/Right Arrow or A/D: Steer", 30, 95, 10, (Color){200, 200, 255, 255});
            DrawText("- C: Toggle Camera View", 30, 115, 10, (Color){200, 200, 255, 255});
            DrawText("- H: Toggle Headlights", 30, 135, 10, (Color){200, 200, 255, 255});
            DrawText("- R: Reset position", 30, 155, 10, (Color){200, 200, 255, 255});
            
            // Stats panel
            DrawRectangleGradientV(GetScreenWidth() - 330, 10, 320, 250, 
                (Color){20, 20, 30, 230}, (Color){40, 40, 60, 200});
            DrawRectangleLines(GetScreenWidth() - 330, 10, 320, 250, (Color){100, 100, 255, 100});
            
            DrawText("Vehicle Telemetry", GetScreenWidth() - 320, 20, 20, (Color){220, 220, 255, 255});
            DrawLine(GetScreenWidth() - 320, 45, GetScreenWidth() - 30, 45, (Color){100, 100, 255, 100});
            
            // Display stats with visual indicators
            char speedText[64];
            sprintf(speedText, "Speed: %.1f km/h", fabsf(carModel.velocity) * 3.6f);
            DrawText(speedText, GetScreenWidth() - 320, 60, 12, WHITE);
            
            // Speed gauge
            float speedPct = fmin(fabsf(carModel.velocity) * 3.6f / 180.0f, 1.0f);
            DrawRectangle(GetScreenWidth() - 320, 80, 310, 15, (Color){50, 50, 70, 255});
            DrawRectangleGradientH(GetScreenWidth() - 320, 80, (int)(310 * speedPct), 15, 
                (Color){50, 150, 255, 255}, (Color){255, 50, 50, 255});
            
            // RPM display
            char rpmText[64];
            sprintf(rpmText, "RPM: %d", (int)carModel.engineRPM);
            DrawText(rpmText, GetScreenWidth() - 320, 105, 12, WHITE);
            
            // RPM gauge with redline
            float rpmPct = carModel.engineRPM / carModel.maxRPM;
            DrawRectangle(GetScreenWidth() - 320, 125, 310, 15, (Color){50, 50, 70, 255});
            
            Color rpmColor = carModel.engineRPM > carModel.redlineRPM ? 
                (Color){255, 50, 50, 255} : (Color){50, 255, 50, 255};
            
            DrawRectangleGradientH(GetScreenWidth() - 320, 125, (int)(310 * rpmPct), 15, 
                (Color){50, 255, 50, 255}, rpmColor);
            
            // Redline indicator
            float redlinePct = carModel.redlineRPM / carModel.maxRPM;
            DrawLine(
                GetScreenWidth() - 320 + (int)(310 * redlinePct), 125,
                GetScreenWidth() - 320 + (int)(310 * redlinePct), 140,
                RED
            );
            
            // Other stats
            char throttleText[64];
            sprintf(throttleText, "Throttle: %d%%", (int)(carModel.throttleInput * 100.0f));
            DrawText(throttleText, GetScreenWidth() - 320, 150, 12, WHITE);
            
            char brakeText[64];
            sprintf(brakeText, "Brake: %d%%", (int)(carModel.brakeInput * 100.0f));
            DrawText(brakeText, GetScreenWidth() - 320, 170, 12, WHITE);
            
            char steeringText[64];
            sprintf(steeringText, "Steering: %.0f°", carModel.steeringAngle * (180.0f / PI));
            DrawText(steeringText, GetScreenWidth() - 320, 190, 12, WHITE);
            
            // Steering indicator
            DrawRectangle(GetScreenWidth() - 220, 190, 200, 10, (Color){60, 60, 80, 255});
            float steeringPct = (carModel.steeringAngle / carModel.maxSteeringAngle) * 0.5f + 0.5f;
            DrawRectangle(GetScreenWidth() - 220 + (int)(steeringPct * 190), 187, 10, 16, ORANGE);
            
            char viewText[64];
            sprintf(viewText, "Camera: %s", firstPersonView ? "First Person" : "Third Person");
            DrawText(viewText, GetScreenWidth() - 320, 210, 12, WHITE);
            
            char lightText[64];
            sprintf(lightText, "Headlights: %s", headlightsOn ? "ON" : "OFF");
            DrawText(lightText, GetScreenWidth() - 320, 230, 12, WHITE);
            
            // FPS counter in corner
            DrawFPS(10, GetScreenHeight() - 30);
            
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
    UnloadModel(carBumperFront);
    UnloadModel(carBumperRear);
    
    // Unload all textures
    UnloadTexture(tireTex);
    UnloadTexture(rimTex);
    UnloadTexture(carTex);
    UnloadTexture(glassTex);
    UnloadTexture(roadTex);
    UnloadTexture(grassTex);
    UnloadTexture(waterTex);
    
    // Unload shader if loaded
    if (useCustomShader) {
        UnloadShader(phongShader);
    }
    
    // Clean up environment objects
    for (auto& obj : environmentObjects) {
        UnloadModel(obj.model);
    }
    
    CloseWindow();        // Close window and OpenGL context
    
    return 0;
}