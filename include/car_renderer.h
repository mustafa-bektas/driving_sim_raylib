#ifndef CAR_RENDERER_H
#define CAR_RENDERER_H

#include "raylib.h"
#include "raymath.h"
#include "car_model.h"
#include "model_factory.h"

class CarRenderer {
public:
    CarRenderer(const ModelFactory& modelFactory);
    void Render(const CarModel& car, float wheelRotation);
    
private:
    const ModelFactory& models;
};

#endif // CAR_RENDERER_H