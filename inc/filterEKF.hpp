#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "TrkUtility.hpp"

class filterEKF
{
    public:
    
    filterEKF();

    ~filterEKF();

    void predict(float dt);
    void calculateInnovation(const float gyroMeasIn[MEAS_SIZE_EKF]);
    void update();

    Eigen::VectorXf predState_;
    Eigen::VectorXf estState_;
    Eigen::VectorXf innovation_;
    Eigen::MatrixXf K_;
    Eigen::MatrixXf predCovariance_;
    Eigen::MatrixXf estCovariance_;
    Eigen::MatrixXf jacobian_;
    Eigen::MatrixXf Q_;
    Eigen::MatrixXf H_;
    Eigen::MatrixXf R_;

    float currentTime_;
    float lastTime_;
    float dt_;
    
};