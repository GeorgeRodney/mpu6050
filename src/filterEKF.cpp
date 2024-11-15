#include "filterEKF.hpp"

filterEKF::filterEKF():
    predState_(STATE_SIZE_EKF),
    estState_(STATE_SIZE_EKF),
    innovation_(MEAS_SIZE_EKF),
    K_(Eigen::MatrixXf::Zero(STATE_SIZE_EKF,MEAS_SIZE_EKF)),
    predCovariance_(Eigen::MatrixXf::Zero(STATE_SIZE_EKF,STATE_SIZE_EKF)),
    estCovariance_(Eigen::MatrixXf::Zero(STATE_SIZE_EKF,STATE_SIZE_EKF)),
    jacobian_(Eigen::MatrixXf::Zero(STATE_SIZE_EKF,STATE_SIZE_EKF)),
    Q_(Eigen::MatrixXf::Zero(STATE_SIZE_EKF,STATE_SIZE_EKF)),
    H_(Eigen::MatrixXf::Zero(MEAS_SIZE_EKF,STATE_SIZE_EKF)),
    R_(Eigen::MatrixXf::Zero(MEAS_SIZE_EKF,MEAS_SIZE_EKF)),
    currentTime_(0.0f),
    lastTime_(0.0f),
    dt_(0.0f)
{
    predState_  << 1.0f, 0.0f, 0.0f, 0.0f,    0.0f, 0.0f, 0.0f,    0.0f, 0.0f, 0.0f;
    estState_   << 1.0f, 0.0f, 0.0f, 0.0f,    0.0f, 0.0f, 0.0f,    0.0f, 0.0f, 0.0f;
    innovation_ << 0.0f, 0.0f, 0.0f;

    Q_(0,0) = 10e-5f;
    Q_(1,1) = 10e-5f;
    Q_(2,2) = 10e-5f;
    Q_(3,3) = 10e-5f;
    Q_(4,4) = 10e-5f;
    Q_(5,5) = 10e-5f;
    Q_(6,6) = 10e-5f;
    Q_(7,7) = 10e-6f;
    Q_(8,8) = 10e-6f;
    Q_(9,9) = 10e-6f;

    H_(0, 4) = 1.0f;
    H_(1, 5) = 1.0f;
    H_(2, 6) = 1.0f;

    R_(0,0) = MEAS_VARIANCE;
    R_(1,1) = MEAS_VARIANCE;
    R_(2,2) = MEAS_VARIANCE;
}

void filterEKF::predict(float dt)
{
    // >----------------------------------------------------
    //  q -> quaternion. 
    //  w -> angular velocity.
    //  a -> angular acceleration.
    // >----------------------------------------------------
    double q0 = estState_(0);
    double q1 = estState_(1);
    double q2 = estState_(2);
    double q3 = estState_(3);
    double w0 = estState_(4);
    double w1 = estState_(5);
    double w2 = estState_(6);
    double a0 = estState_(7);
    double a1 = estState_(8);
    double a2 = estState_(9);

    jacobian_ << 1.0, -0.5*(w0*dt+a0*dt*dt), -0.5*(w1*dt+a1*dt*dt), -0.5*(w2*dt+a2*dt*dt), -0.5*(q1*dt), -0.5*(q2*dt), -0.5*(q3*dt), -0.5*(q1*dt*dt), -0.5*(q2*dt*dt), -0.5*(q3*dt*dt),
                0.5*(w0*dt+a0*dt*dt), 1.0, 0.5*(w2*dt+a2*dt*dt), 0.5*(w1*dt+a1*dt*dt), 0.5*(q0*dt), 0.5*(q3*dt), 0.5*(q2*dt), 0.5*(q0*dt*dt), 0.5*(q3*dt*dt), 0.5*(q2*dt*dt),
                0.5*(w1*dt+a1*dt*dt), -0.5*(w2*dt+a2*dt*dt), 1.0, 0.5*(w0*dt+a0*dt*dt), 0.5*(q3*dt), 0.5*(q0*dt), -0.5*(q1*dt), 0.5*(q3*dt*dt), 0.5*(q0*dt*dt), -0.5*(q1*dt*dt),
                0.5*(w2*dt+a2*dt*dt), 0.5*(w1*dt+a1*dt*dt), -0.5*(w0*dt+a0*dt*dt), 1.0, -0.5*(q2*dt), 0.5*(q1*dt), 0.5*(q0*dt), -0.5*(q2*dt*dt), 0.5*(q1*dt*dt), 0.5*(q0*dt*dt),
                0.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 1.0;

    // >----------------------------------------------------
    //  Process Model 
    //  qPred = qEst * qDelta.
    //  wPred = wEst + aEst*dt.
    //  aPred = aEst.
    // >----------------------------------------------------
    Eigen::Quaterniond deltaQuat;
    deltaQuat.w() = 1.0;
    deltaQuat.x() = 0.5 * (w0 * dt + 0.5 * a0 * dt * dt);
    deltaQuat.y() = 0.5 * (w1 * dt + 0.5 * a1 * dt * dt);
    deltaQuat.z() = 0.5 * (w2 * dt + 0.5 * a2 * dt * dt);

    Eigen::Quaterniond estQuat(q0, q1, q2, q3);

    Eigen::Quaterniond predQuat = estQuat * deltaQuat;
    predQuat.normalize();

    predState_(0) = predQuat.w();
    predState_(1) = predQuat.x();
    predState_(2) = predQuat.y();
    predState_(3) = predQuat.z();

    predState_(4) = w0 + a0*dt;
    predState_(5) = w1 + a1*dt;
    predState_(6) = w2 + a2*dt;

    predState_(7) = a0;
    predState_(8) = a1;
    predState_(9) = a2;

    // >----------------------------------------------------
    //  Predict Covariance.
    //
    //  P(k|k-1) = J(k) * P(k-1|k-1) * J(k).transpose() + Q()
    //
    // >----------------------------------------------------
    predCovariance_ = jacobian_ * estCovariance_ * jacobian_.transpose() + Q_;
}

void filterEKF::calculateInnovation(const float gyroMeasIn[3])
{
    innovation_(0) = gyroMeasIn[0] - predState_(4);
    innovation_(1) = gyroMeasIn[1] - predState_(5);
    innovation_(2) = gyroMeasIn[2] - predState_(6);

    float magDiff = std::sqrt(std::pow(innovation_(0),2) + std::pow(innovation_(1),2) + std::pow(innovation_(2),2));

    printf("Innovation differance: %f\n", magDiff);
}

void filterEKF::update()
{
    // >------------------------------------------------------------
    //  Kalman Gain.
    //  
    //  K(k) = P(k|k-1) * H.transpose() * [H * P(k|k-1) * H.transpose() + R].inverse()
    // 
    // >------------------------------------------------------------
    Eigen::MatrixXf S = H_ * predCovariance_ * H_.transpose() + R_;

    K_ = predCovariance_ * H_.transpose() * S.inverse();

    // >------------------------------------------------------------
    //  Update Estimate.
    //  
    //  X(k|k) = X(k|k-1) + K(k) * innovation(k)
    //  
    // >------------------------------------------------------------
    estState_ = predState_ + K_ * innovation_;
    Eigen::Quaterniond normalizeQuat(estState_(0), estState_(1), estState_(2), estState_(3));
    normalizeQuat.normalize();
    estState_(0) = normalizeQuat.w();
    estState_(1) = normalizeQuat.x();
    estState_(2) = normalizeQuat.y();
    estState_(3) = normalizeQuat.z();

    // >------------------------------------------------------------
    //  Update Covariance.
    //  
    //  P(k|k) = [I - K(k) * H.transpose()] * P(k|k-1)
    //  
    // >------------------------------------------------------------
    estCovariance_ = (Eigen::MatrixXf::Identity(STATE_SIZE_EKF,STATE_SIZE_EKF) - K_ * H_) * predCovariance_;
}