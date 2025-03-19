#ifndef UI_RENDERER_H
#define UI_RENDERER_H

#include "raylib.h"
#include "car_model.h"

class UIRenderer {
public:
    void RenderUI(const CarModel& car);
};

#endif // UI_RENDERER_H