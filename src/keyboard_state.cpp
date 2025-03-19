#include "../include/keyboard_state.h"

KeyboardState::KeyboardState() {
    // Initialize all keys to false
    up = down = left = right = w = s = a = d = r = false;
}

void KeyboardState::Update() {
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

bool KeyboardState::IsThrottlePressed() const {
    return up || w;
}

bool KeyboardState::IsBrakePressed() const {
    return down || s;
}

bool KeyboardState::IsSteeringLeftPressed() const {
    return left || a;
}

bool KeyboardState::IsSteeringRightPressed() const {
    return right || d;
}

bool KeyboardState::IsResetPressed() const {
    return r;
}