#include "OrientationFilter.hpp"


OrientationFilter::OrientationFilter():
    predState_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    estState_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
}

void OrientationFilter::predict(float dt)
{   
}

void OrientationFilter::update(float measNoise)
{
}