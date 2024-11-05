#include "OrientationFilter.hpp"


OrientationFilter::OrientationFilter():
    predState_{0.0f, 1.0f, 0.0f, 0.0f, 3.0f, 3.0f, 3.0f},
    estState_{0.0f, 1.0f, 0.0f, 0.0f, 3.0f, 3.0f, 3.0f},
    pred_Var_(Eigen::MatrixXd::Zero(STATE_SIZE,STATE_SIZE)),
    est_Var_(Eigen::MatrixXd::Zero(STATE_SIZE,STATE_SIZE)),
    lambda_(0.0),
    W_mean_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
    W_cov_(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
    angular_velocity_noise_var(5.0),
    angular_velocity_noise_stddev(std::sqrt(angular_velocity_noise_var))
{
    pred_Var_(0,0) = 0.1;
    pred_Var_(1,1) = 0.1;
    pred_Var_(2,2) = 0.1;
    pred_Var_(3,3) = 0.1;
    pred_Var_(4,4) = 1;
    pred_Var_(5,5) = 1;
    pred_Var_(6,6) = 1;

    est_Var_(0,0) = 0.1;
    est_Var_(1,1) = 0.1;
    est_Var_(2,2) = 0.1;
    est_Var_(3,3) = 0.1;
    est_Var_(4,4) = 1;
    est_Var_(5,5) = 1;
    est_Var_(6,6) = 1;

    std::normal_distribution<float> noise_dist(0.0, angular_velocity_noise_stddev);

}

void OrientationFilter::predict(float dt)
{   
    // GENERATE SIGMA POINTS
    calcWeights();
    Eigen::MatrixXd sigma(Eigen::MatrixXd::Zero(STATE_SIZE,15));
    Eigen::MatrixXd sigma_prime(Eigen::MatrixXd::Zero(STATE_SIZE,15));
    sigma.col(0) = estState_; 

    // est_Var_ = sqrt_P * sqrt_P.Transpose()
    // therefore, sqrt_P is essentially the square root of the variance matrix
    Eigen::MatrixXd sqrt_P = est_Var_.llt().matrixL();
    float scaleFactor = std::sqrt(STATE_SIZE + lambda_);   

    for (uint16_t posIdx = 1; posIdx <= 7; ++posIdx)
    {   
        for (uint16_t row = 0; row < STATE_SIZE; ++row)
        {
            sigma(row, posIdx) = estState_(row) + scaleFactor * sqrt_P(row, row);
            printf("sigma(%d,%d): %f", row, posIdx, sigma(row, posIdx));
            printf("\n");
        }
    }

    printf("\n\n\n");

    for (uint16_t negIdx = 8; negIdx <= 14; ++negIdx)
    {
        for (uint16_t row = 0; row < STATE_SIZE; ++row)
        {
            sigma(row, negIdx) = estState_(row) - scaleFactor * sqrt_P(row, row);
            printf("sigma(%d,%d): %f", row, negIdx, sigma(row, negIdx)); 
            printf("\n");  
        }
    }

    // PROPOGATE SIGMA THROUGH TRANSFORM
    Eigen::Quaterniond estQuat;
    estQuat.w() = estState_(0);
    estQuat.x()= estState_(1);
    estQuat.y() = estState_(2);
    estQuat.z() = estState_(3);

    for (uint16_t col = 0; col < 15; ++col)
    {
        // Normalize the quat
        sigma.col(col).head(4).normalize();
        Eigen::Quaterniond deltaQ;
        Eigen::Quaterniond newQ;
        
        deltaQ.w() = 1.0;
        deltaQ.x() = 0.5 * sigma(4, col) * dt;
        deltaQ.y() = 0.5 * sigma(5, col) * dt;
        deltaQ.z() = 0.5 * sigma(6, col) * dt;

        newQ = estQuat * deltaQ;
        sigma_prime.col(col)(0) = newQ.w();
        sigma_prime.col(col)(1) = newQ.x();
        sigma_prime.col(col)(2) = newQ.y();
        sigma_prime.col(col)(3) = newQ.z();

        float noise_wx = noise_dist(gen);
        float noise_wy = noise_dist(gen);
        float noise_wz = noise_dist(gen);

        sigma_prime(4, col) = sigma(4, col) + noise_wx * dt; // Angular velocity x component
        sigma_prime(5, col) = sigma(5, col) + noise_wy * dt; // Angular velocity y component
        sigma_prime(6, col) = sigma(6, col) + noise_wz * dt; // Angular velocity z component
    }



    // PREDICT VARIANCE 
}

void OrientationFilter::update(float measNoise)
{
}

void OrientationFilter::calcWeights()
{
    float alpha = 1e-3;
    float beta = 2.0;
    float kappa = 0.0;

    // Calculate lambda
    lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
    printf("Lamda: %f", lambda_);

    // Compute weights
    W_mean_(0) = lambda_ / (STATE_SIZE + lambda_);
    W_cov_(0) = lambda_ / (STATE_SIZE + lambda_) + (1 - alpha * alpha + beta);

    for (int i = 1; i < 2 * STATE_SIZE + 1; ++i) {
        W_mean_(i) = 1.0 / (2 * (STATE_SIZE + lambda_));
        W_cov_(i) = 1.0 / (2 * (STATE_SIZE + lambda_));
    }
}