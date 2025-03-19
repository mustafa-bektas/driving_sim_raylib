#ifndef MODEL_FACTORY_H
#define MODEL_FACTORY_H

#include "raylib.h"
#include "raymath.h"
#include <vector>

class ModelFactory {
public:
    ModelFactory();
    ~ModelFactory();
    
    // Initialize all car models
    void Initialize();
    
    // Unload all models
    void Unload();
    
    // Get models
    Model GetCarBodyModel() const { return carBody; }
    Model GetCarHoodModel() const { return carHood; }
    Model GetCarTrunkModel() const { return carTrunk; }
    Model GetCarCabinModel() const { return carCabin; }
    Model GetCarRoofModel() const { return carRoof; }
    Model GetWheelModel() const { return wheelModel; }
    Model GetHeadlightModel() const { return headlight; }
    Model GetTaillightModel() const { return taillight; }
    Model GetBumperModel() const { return bumper; }
    
private:
    // Function to create a wheel mesh
    Model CreateWheelModel();
    
    // Function to create a simple box model with a given color
    Model CreateBoxModel(float width, float height, float length, Color color);
    
    // Car part models
    Model carBody;
    Model carHood;
    Model carTrunk;
    Model carCabin;
    Model carRoof;
    Model wheelModel;
    Model headlight;
    Model taillight;
    Model bumper;
};

#endif // MODEL_FACTORY_H