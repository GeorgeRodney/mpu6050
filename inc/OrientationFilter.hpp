#include <Eigen/Dense>
#include <Eigen/Geometry>

class OrientationFilter
{
    public:

    OrientationFilter();

    ~OrientationFilter();

    void predict(float dt);

    void update(float measNoise);

    Eigen::Matrix<double, 7, 1> state_;
    Eigen::Quaterniond tempQuat_;
    
};