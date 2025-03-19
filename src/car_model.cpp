#include "../include/car_model.h"

CarModel::CarModel() {
    // Constructor - values are already initialized in the header
    Reset();
}

void CarModel::Reset() {
    // Reset position and orientation
    position = (Vector3){ 0.0f, 0.0f, 0.0f };
    heading = PI / 2.0f; // Forward
    
    // Reset physics state
    velocity = 0.0f;
    lateralVelocity = 0.0f;
    yawRate = 0.0f;
    engineRPM = idleRPM;
    wheelAngularVelocity = 0.0f;
    throttleInput = 0.0f;
    brakeInput = 0.0f;
    steeringAngle = 0.0f;
}

float CarModel::GetEngineTorque(float rpm, float throttle) {
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

float CarModel::GetTireForce(float slipRatio, float normalForce) {
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

float CarModel::GetWheelRotation(float deltaTime) {
    static float wheelRotation = 0.0f;
    wheelRotation += wheelAngularVelocity * deltaTime;
    return wheelRotation;
}

void CarModel::Update(float deltaTime, bool throttlePressed, bool brakePressed, bool steerLeftPressed, bool steerRightPressed) {
    // Handle throttle input (accelerator pedal)
    if (throttlePressed) {
        throttleInput += 2.0f * deltaTime; // Smoother throttle application
        throttleInput = fmin(throttleInput, 1.0f);
        brakeInput = 0.0f; // Release brakes when accelerating
    } else {
        throttleInput -= 3.0f * deltaTime; // Throttle returns to zero when released
        throttleInput = fmax(throttleInput, 0.0f);
    }
    
    // Brake input
    if (brakePressed) {
        brakeInput += 3.0f * deltaTime; // Smoother brake application
        brakeInput = fmin(brakeInput, 1.0f);
        throttleInput = 0.0f; // Release throttle when braking
    } else {
        brakeInput -= 4.0f * deltaTime; // Brakes release quickly
        brakeInput = fmax(brakeInput, 0.0f);
    }
    
    // Steering input
    if (steerLeftPressed) {
        steeringAngle -= steeringSpeed * deltaTime;
        steeringAngle = fmax(steeringAngle, -maxSteeringAngle);
    } else if (steerRightPressed) {
        steeringAngle += steeringSpeed * deltaTime;
        steeringAngle = fmin(steeringAngle, maxSteeringAngle);
    } else {
        // Return steering to center gradually
        if (fabsf(steeringAngle) > 0.01f) {
            steeringAngle -= (steeringAngle > 0 ? 1.0f : -1.0f) * steeringSpeed * 0.5f * deltaTime;
        } else {
            steeringAngle = 0.0f;
        }
    }
    
    // More realistic engine RPM management
    // RPM rises with throttle, falls without throttle
    if (throttleInput > 0.0f) {
        // Engine revs up when throttle is applied
        float rpmIncreaseRate = 2000.0f; // RPM per second base rate
        
        // Lower RPM increase as we approach redline
        float rpmHeadroom = (maxRPM - engineRPM) / (maxRPM - idleRPM);
        rpmHeadroom = fmax(0.1f, rpmHeadroom); // Prevent division by zero
        
        // Apply throttle and headroom factors
        float actualRpmIncrease = rpmIncreaseRate * throttleInput * rpmHeadroom * deltaTime;
        engineRPM += actualRpmIncrease;
        
        // Limit RPM to maximum
        engineRPM = fmin(engineRPM, maxRPM);
    } else {
        // Engine slows down when throttle is released
        engineRPM -= 1500.0f * deltaTime;
        engineRPM = fmax(engineRPM, idleRPM);
    }
    
    // Calculate weight distribution (simplified)
    const float totalWeight = mass * 9.81f; // N (mass * gravity)
    const float rearAxleWeight = totalWeight * 0.5f; // Simplified 50/50 weight distribution
    
    // Calculate engine torque based on throttle and RPM
    engineTorque = GetEngineTorque(engineRPM, throttleInput);
    
    // Calculate wheel torque through transmission
    const float totalGearRatio = gearRatio * differentialRatio;
    
    // Add clutch slippage simulation
    float effectiveClutchEngagement = 0.2f; // Minimum engagement
    
    // If moving and throttle applied, increase clutch engagement
    if (throttleInput > 0.0f) {
        // Clutch engagement factor increases with speed (simulates clutch slip at low speeds)
        effectiveClutchEngagement += fmin(0.8f, (fabsf(velocity) / 10.0f) * 0.8f);
        
        // Additional "launch" help at very low speeds
        if (fabsf(velocity) < 0.5f) {
            // When starting from a stop, give extra initial torque
            wheelAngularVelocity += throttleInput * 2.0f * deltaTime;
        }
    }
    
    // Apply clutch engagement to torque transfer
    wheelTorque = engineTorque * totalGearRatio * transmissionEfficiency * effectiveClutchEngagement;
    
    // Calculate tractive force at driven wheels - ensure it's positive when accelerating
    tractionForce = fmax(0.0f, wheelTorque / wheelRadius); // Ensure traction force is positive when under power
    
    // Calculate wheelSpeed for slip calculations
    const float wheelSpeed = wheelAngularVelocity * wheelRadius;
    
    // Calculate longitudinal slip ratio with better handling of low speeds
    float tireSlipRatio = 0.0f;
    
    if (fabsf(velocity) < 0.5f) {
        // Special case for starting from standstill - small positive slip
        if (throttleInput > 0.0f) {
            tireSlipRatio = 0.05f; // Small positive slip to get the car moving
        }
    } else {
        // Regular slip calculation for normal driving
        // Prevent division by zero when calculating slip
        float denominator = fmax(fabsf(velocity), 0.5f);
        tireSlipRatio = (wheelSpeed - velocity) / denominator;
        
        // Limit slip ratio to reasonable values
        tireSlipRatio = fmax(-0.5f, fmin(0.5f, tireSlipRatio));
    }
    
    // Calculate longitudinal tire force using slip model
    const float longitudinalForce = GetTireForce(tireSlipRatio, rearAxleWeight);
    
    // Apply traction force limit based on tire grip
    // Always keep tractionForce positive when throttle is applied for forward motion
    float limitedTractionForce;
    
    if (throttleInput > 0.01f) {
        // When accelerating, ensure positive force
        limitedTractionForce = fmin(tractionForce, fabsf(longitudinalForce));
    } else {
        // When not accelerating, use the tire model
        limitedTractionForce = fmin(
            fabsf(tractionForce),
            fabsf(longitudinalForce)
        ) * (tractionForce >= 0.0f ? 1.0f : -1.0f);
    }
    
    // Add a minimum starting force when throttle is applied to simulate clutch engagement
    if (throttleInput > 0.05f && fabsf(velocity) < 0.5f) {
        const float startingForce = 600.0f * throttleInput; // Reduced from original 1000.0f for smoother start
        limitedTractionForce = fmax(limitedTractionForce, startingForce);
    }
    
    // Calculate drag force (increases with square of velocity)
    dragForce = 0.5f * dragCoefficient * velocity * fabsf(velocity);
    
    // Calculate rolling resistance with speed-dependent factor
    // At very low speeds, reduce rolling resistance to help get moving
    float effectiveRollingResistance = rollingResistance;
    if (fabsf(velocity) < 1.0f) {
        // Gradually increase rolling resistance with speed
        effectiveRollingResistance *= fabsf(velocity);
    }
    
    const float rollingForce = effectiveRollingResistance * totalWeight * (velocity >= 0.0f ? -1.0f : 1.0f);
    
    // Calculate braking force
    const float brakeForce = brakeInput * brakeForceMax * (velocity <= 0.0f ? 1.0f : -1.0f);
    
    // Calculate net longitudinal force
    // Note: dragForce and rollingForce naturally have the correct sign (opposite to velocity)
    // limitedTractionForce has the correct sign based on throttle
    // brakeForce has the correct sign (negative for positive velocity)
    const float netLongitudinalForce = limitedTractionForce + dragForce + rollingForce + brakeForce;
    lastAppliedForce = netLongitudinalForce; // Store for debugging
    
    // Calculate longitudinal acceleration
    const float longitudinalAcceleration = netLongitudinalForce / mass;
    
    // Calculate effective steering angle with speed-dependent adjustment
    const float speedFactor = fmin(1.0f, fabsf(velocity) / 5.0f);
    const float effectiveSteeringAngle = steeringAngle * (1.0f - 0.3f * speedFactor);
    
    // Bicycle model calculations for slip angles
    // For front wheels (steering wheels)
    const float frontSlipAngle = atan2f(
        lateralVelocity + yawRate * (wheelbase/2.0f), 
        fabsf(velocity) + 0.001f
    ) - effectiveSteeringAngle;
    
    // For rear wheels (non-steering)
    const float rearSlipAngle = atan2f(
        lateralVelocity - yawRate * (wheelbase/2.0f), 
        fabsf(velocity) + 0.001f
    );
    
    // Calculate forces using slip angles and the cornering stiffness
    const float frontLateralForce = -corneringStiffness * frontSlipAngle;
    const float rearLateralForce = -corneringStiffness * rearSlipAngle;
    
    // Limit lateral forces based on weight and friction
    const float maxFrontLateralForce = totalWeight * 0.5f * tireFrictionCoefficient;
    const float maxRearLateralForce = totalWeight * 0.5f * tireFrictionCoefficient;
    
    const float limitedFrontLateralForce = fmax(-maxFrontLateralForce, 
                                            fmin(maxFrontLateralForce, frontLateralForce));
    const float limitedRearLateralForce = fmax(-maxRearLateralForce, 
                                           fmin(maxRearLateralForce, rearLateralForce));
    
    // Calculate total lateral force
    const float totalLateralForce = limitedFrontLateralForce + limitedRearLateralForce;
    
    // Calculate lateral acceleration
    const float lateralAcceleration = totalLateralForce / mass;
    
    // Calculate yaw moment from steering forces
    // Positive moment turns clockwise
    const float yawMoment = (limitedFrontLateralForce * (wheelbase/2.0f)) - 
                         (limitedRearLateralForce * (wheelbase/2.0f));
    
    // Calculate yaw acceleration (rotation)
    const float momentOfInertia = mass * (wheelbase * wheelbase) / 12.0f; // Simplified
    const float yawAcceleration = yawMoment / momentOfInertia;
    
    // Update velocities using forces (semi-implicit Euler integration)
    velocity += longitudinalAcceleration * deltaTime;
    lateralVelocity += lateralAcceleration * deltaTime;
    yawRate += yawAcceleration * deltaTime;
    
    // Add extra turning force at low speeds to improve maneuverability
    if (fabsf(velocity) > 0.1f) {
        // Low-speed steering boost - higher effect at lower speeds
        const float steeringBoostFactor = fmax(0.0f, 1.0f - (fabsf(velocity) / 5.0f));
        if (steeringBoostFactor > 0.0f) {
            // Apply boost only when car is moving and steering is applied
            yawRate += effectiveSteeringAngle * steeringBoostFactor * 1.5f * deltaTime * 
                      (velocity >= 0.0f ? 1.0f : -1.0f);
        }
    }
    
    // Apply velocity damping to simulate tire slip and natural stabilization
    lateralVelocity *= 0.95f; // Lateral slip damping
    yawRate *= 0.95f; // Yaw damping (simulates stabilizing forces)
    
    // Update wheel angular velocity based on engine and braking
    // When braking, wheels slow down faster than the car
    const float wheelTorqueNet = wheelTorque - (brakeForce * wheelRadius);
    const float wheelAcceleration = wheelTorqueNet / wheelMomentOfInertia;
    wheelAngularVelocity += wheelAcceleration * deltaTime;
    
    // Update position based on velocity
    // Forward is -Z, right is +X in our coordinate system
    position.z += velocity * cosf(heading) * deltaTime;
    position.x -= velocity * sinf(heading) * deltaTime;
    
    // Add lateral velocity component
    position.z += lateralVelocity * sinf(heading) * deltaTime;
    position.x += lateralVelocity * cosf(heading) * deltaTime;
    
    // Update heading based on yaw rate
    heading += yawRate * deltaTime;
    
    // Calculate a simulated clutch engagement factor
    // When starting from a stop or at low speeds, the clutch slips more
    float clutchEngagement = 0.0f;
    if (throttleInput > 0.0f) {
        // Clutch engagement increases with speed
        clutchEngagement = fmin(1.0f, fabsf(velocity) / 5.0f + 0.2f);
    }
    
    // Calculate wheel RPM based on vehicle speed (regardless of engine)
    float wheelRPM = (velocity / wheelRadius) * (60.0f / (2.0f * PI));
    
    // Synchronize wheel angular velocity with the vehicle's actual velocity
    wheelAngularVelocity = (velocity / wheelRadius);
    
    // Apply some "engine braking" proportional to the difference between 
    // engine RPM and what the wheels would need at current speed
    if (throttleInput < 0.01f && fabsf(velocity) > 0.5f) {
        float desiredWheelRPM = wheelRPM;
        float engineBrakingForce = 500.0f * (velocity > 0 ? 1.0f : -1.0f);
        
        // Apply engine braking as a small force that slows the car
        const float engineBrakingAccel = engineBrakingForce / mass;
        velocity -= engineBrakingAccel * deltaTime;
    }
}