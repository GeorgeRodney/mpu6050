#include "OrientationFilter.hpp"


OrientationFilter::OrientationFilter():
    state_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
{
}

void OrientationFilter::predict(float dt)
{
    state_(0) = 1.0;
    state_(1) = 2.0;
    state_(2) = 3.0;
    state_(3) = 4.0;

}

void OrientationFilter::update(float measNoise)
{
    state_(0) = 2.0;
    state_(1) = 4.0;
    state_(2) = 6.0;
    state_(3) = 8.0;
}