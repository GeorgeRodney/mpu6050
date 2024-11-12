#include "OrientationFilter.hpp"


OrientationFilter::OrientationFilter():
    pred_State_(7),
    est_State_(7),
    innovation_(3),
    pred_Var_(Eigen::MatrixXd::Zero(STATE_SIZE,STATE_SIZE)),
    est_Var_(Eigen::MatrixXd::Zero(STATE_SIZE,STATE_SIZE)),
    innovation_Var_(Eigen::MatrixXd::Zero(MEAS_SIZE,MEAS_SIZE)),
    pred_Meas_(7),
    est_Meas_(7),
    lambda_(0.0),
    W_mean_(Eigen::MatrixXd::Zero(15,1)),
    W_cov_(Eigen::MatrixXd::Zero(15,1)),
    state_meas_cov_(Eigen::MatrixXd::Zero(STATE_SIZE,MEAS_SIZE)),
    K(Eigen::MatrixXd::Zero(STATE_SIZE,MEAS_SIZE)),
    predicted_sigma_(Eigen::MatrixXd::Zero(STATE_SIZE,15)),
    measurement_sigma_(Eigen::MatrixXd::Zero(MEAS_SIZE,15)),
    measurement_Var_(Eigen::MatrixXd::Zero(MEAS_SIZE,MEAS_SIZE)),
    currentTime_(0.0),
    lastTime_(0.0),
    dt_(0.0),
    angular_velocity_noise_var(5.0),
    angular_velocity_noise_stddev(std::sqrt(angular_velocity_noise_var)),
    measurement_noise_matrix_(Eigen::MatrixXd::Zero(MEAS_SIZE,MEAS_SIZE))
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

    std::normal_distribution<double> noise_dist(0.0, angular_velocity_noise_stddev);

    measurement_noise_matrix_(0,0) = angular_velocity_noise_var;
    measurement_noise_matrix_(1,1) = angular_velocity_noise_var;
    measurement_noise_matrix_(2,2) = angular_velocity_noise_var;

    pred_State_ << 1, 1, 0, 0, 0, 0, 0;
    est_State_ << 1, 1, 0, 0, 0, 0, 0;
}

void OrientationFilter::predict(double dt)
{   
    Eigen::MatrixXd sigma(Eigen::MatrixXd::Zero(STATE_SIZE,15));
    Eigen::MatrixXd sigma_prime(Eigen::MatrixXd::Zero(STATE_SIZE,15));
    Eigen::Quaterniond estQuat;
    Eigen::Quaterniond deltaQ;
    Eigen::Quaterniond newQ;

    Eigen::MatrixXd sqrt_P = est_Var_.llt().matrixL();

    double scaleFactor = std::sqrt(STATE_SIZE + lambda_);   

    // GENERATE SIGMA POINTS
    sigma.col(0) = est_State_; 
    for (uint16_t posIdx = 1; posIdx <= STATE_SIZE; ++posIdx)
    {   
        for (uint16_t row = 0; row < STATE_SIZE; ++row)
        {
            sigma(row, posIdx) = est_State_(row) + scaleFactor * sqrt_P(row, row);
            // printf("sigma(%d,%d): %f", row, posIdx, sigma(row, posIdx));
            // printf("\n");
        }
    }

    for (uint16_t negIdx = STATE_SIZE+1; negIdx <= 2*STATE_SIZE; ++negIdx)
    {
        for (uint16_t row = 0; row < STATE_SIZE; ++row)
        {
            sigma(row, negIdx) = est_State_(row) - scaleFactor * sqrt_P(row, row);
            // printf("sigma(%d,%d): %f", row, negIdx, sigma(row, negIdx)); 
            // printf("\n");  
        }
    }

    printf("Sigma\n");
    for (uint16_t row = 0; row < STATE_SIZE; ++row)
    {
        for (uint16_t col = 0; col < 2*STATE_SIZE+1; ++col)
        {
            printf("%d, ", sigma(row,col));
        }
        printf("\n");
    }

    // PROPOGATE SIGMA THROUGH TRANSFORM
    estQuat.w() = est_State_(0);
    estQuat.x() = est_State_(1);
    estQuat.y() = est_State_(2);
    estQuat.z() = est_State_(3);

    for (uint16_t col = 0; col < (2*STATE_SIZE+1); ++col)
    {
        // Normalize the quat
        sigma.col(col).head(4).normalize();
        
        deltaQ.w() = 1.0;
        deltaQ.x() = 0.5 * sigma(4, col) * dt;
        deltaQ.y() = 0.5 * sigma(5, col) * dt;
        deltaQ.z() = 0.5 * sigma(6, col) * dt;

        newQ = estQuat * deltaQ;
        sigma_prime.col(col)(0) = newQ.w();
        sigma_prime.col(col)(1) = newQ.x();
        sigma_prime.col(col)(2) = newQ.y();
        sigma_prime.col(col)(3) = newQ.z();

        double noise_wx = noise_dist(gen);
        double noise_wy = noise_dist(gen);
        double noise_wz = noise_dist(gen);

        sigma_prime(4, col) = sigma(4, col) + noise_wx * dt; // Angular velocity x component
        sigma_prime(5, col) = sigma(5, col) + noise_wy * dt; // Angular velocity y component
        sigma_prime(6, col) = sigma(6, col) + noise_wz * dt; // Angular velocity z component
    }

    printf("Sigma Prime\n");
    for (uint16_t row = 0; row < STATE_SIZE; ++row)
    {
        for (uint16_t col = 0; col < 2*STATE_SIZE+1; ++col)
        {
            printf("%d, ", sigma_prime(row,col));
        }
        printf("\n");
    }

    predicted_sigma_ = sigma_prime;

    // PREDICT STATE VECTOR
    pred_State_.setZero();
    for (uint16_t sigmaIdx = 0; sigmaIdx < (2*STATE_SIZE+1); ++sigmaIdx)
    {
        pred_State_ += W_mean_(sigmaIdx) * sigma_prime.col(sigmaIdx);
    }

    // PREDICT COVARIANCE MATRIX
    pred_Var_.setZero();
    for (int idx = 0; idx < 15; ++idx)
    {
        Eigen::VectorXd diff = sigma_prime.col(idx) - pred_State_;
        pred_Var_ += W_cov_(idx) * (diff * diff.transpose());
    }
}

// >-----------------------------------------------------------------------------------
//  Predict the measurement.
//  Inputs: Predicted State. Predicted Covariance.
//  Outputs: Predicted measurement state
// >-----------------------------------------------------------------------------------

void OrientationFilter::predictMeasurement()
{
    Eigen::MatrixXd sigma(Eigen::MatrixXd::Zero(STATE_SIZE,15));
    Eigen::MatrixXd sigma_prime(Eigen::MatrixXd::Zero(MEAS_SIZE,15));
        
    Eigen::MatrixXd sqrt_P = pred_Var_.llt().matrixL();

    double scaleFactor = std::sqrt(STATE_SIZE + lambda_);   

    // GENERATE SIGMA POINTS
    sigma.col(0) = pred_State_; 
    for (uint16_t posIdx = 1; posIdx <= STATE_SIZE; ++posIdx)
    {   
        for (uint16_t row = 0; row < STATE_SIZE; ++row)
        {
            sigma(row, posIdx) = pred_State_(row) + scaleFactor * sqrt_P(row, row);
            // printf("sigma(%d,%d): %f", row, posIdx, sigma(row, posIdx));
            // printf("\n");
        }
    }

    for (uint16_t negIdx = STATE_SIZE+1; negIdx <= 2*STATE_SIZE; ++negIdx)
    {
        for (uint16_t row = 0; row < STATE_SIZE; ++row)
        {
            sigma(row, negIdx) = pred_State_(row) - scaleFactor * sqrt_P(row, row);
            // printf("sigma(%d,%d): %f", row, negIdx, sigma(row, negIdx)); 
            // printf("\n");  
        }
    }

    // PROPOGATE SIGMA THROUGH TRANSFORM
    for (uint16_t col = 0; col < (2*STATE_SIZE+1); ++col)
    {
        sigma_prime(0, col) = sigma(0, col);
        sigma_prime(1, col) = sigma(1, col);
        sigma_prime(2, col) = sigma(2, col);
    }

    measurement_sigma_ = sigma_prime;

    // PREDICT MEAS VECTOR
    pred_Meas_.setZero();
    for (uint16_t sigmaIdx = 0; sigmaIdx < (2*STATE_SIZE+1); ++sigmaIdx)
    {
        pred_Meas_ += W_mean_(sigmaIdx) * sigma_prime.col(sigmaIdx);
    }

    // PREDICT INNOVATION COVARIANCE
    innovation_Var_.setZero();
    for (int idx = 0; idx < 15; ++idx)
    {
        Eigen::VectorXd diff = sigma_prime.col(idx) - pred_Meas_;
        innovation_Var_ += W_cov_(idx) * (diff * diff.transpose());
    }
}

void OrientationFilter::calculateInnovation(const float gyroMeasIn[3])
{
    for (uint8_t idx = 0; idx < MEAS_SIZE; ++idx)
    {
        innovation_(idx) = gyroMeasIn[idx] - pred_Meas_(idx);
    }
}

void OrientationFilter::update()
{  
    // State and measurement covariance
    state_meas_cov_.setZero();
    for (int idx = 0; idx < 15; ++idx)
    {   
        Eigen::VectorXd diffState = predicted_sigma_.col(idx) - pred_State_;
        Eigen::VectorXd diffMeas = measurement_sigma_.col(idx) - pred_Meas_;
        state_meas_cov_ += W_cov_(idx) * (diffState * diffMeas.transpose());
    }

    measurement_Var_.setZero();
    for (int idx = 0; idx < 15; ++idx)
    {   
        Eigen::VectorXd diffMeas = measurement_sigma_.col(idx) - pred_Meas_;
        measurement_Var_ += W_cov_(idx) * (diffMeas * diffMeas.transpose());
    }

    measurement_Var_ += measurement_noise_matrix_;

    // K = P_xz * S_inv
    K = state_meas_cov_ * measurement_Var_.inverse();
    
    // X(+) = X(-) + K * innovation
    est_State_ = pred_State_ + K * innovation_;

    // P(+) = P(-) - K * S * K_t
    est_Var_ = pred_Var_ - K * measurement_Var_ *  K.transpose();
}

void OrientationFilter::calcWeights()
{
    double alpha = 1.0;
    double beta = 0.0;
    double kappa = 3 - STATE_SIZE;

    // Calculate lambda
    lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
    printf("lambda: %f\n", lambda_);

    // Compute weights
    W_mean_(0) = lambda_ / (STATE_SIZE + lambda_);
    W_cov_(0) = lambda_ / (STATE_SIZE + lambda_) + (1 - alpha * alpha + beta);

    for (int i = 1; i < 2 * STATE_SIZE + 1; ++i) {
        W_mean_(i) = 1.0 / (2 * (STATE_SIZE + lambda_));
        W_cov_(i) = 1.0 / (2 * (STATE_SIZE + lambda_));
        printf("W_mean: %f\n", W_mean_(i));
    }
}