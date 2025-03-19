#ifndef KEYBOARD_STATE_H
#define KEYBOARD_STATE_H

#include "raylib.h"

// Keyboard input handler
class KeyboardState {
public:
    KeyboardState();
    void Update();
    
    bool IsThrottlePressed() const;
    bool IsBrakePressed() const;
    bool IsSteeringLeftPressed() const;
    bool IsSteeringRightPressed() const;
    bool IsResetPressed() const;
    
private:
    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;
    bool w = false;
    bool s = false;
    bool a = false;
    bool d = false;
    bool r = false;
};

#endif // KEYBOARD_STATE_H