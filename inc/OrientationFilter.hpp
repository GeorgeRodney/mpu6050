#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#define STATE_SIZE 7

class OrientationFilter
{
    public:

    OrientationFilter();

    ~OrientationFilter();

    void predict(float dt);

    void update(float measNoise);

    void calcWeights();

    Eigen::Matrix<float, 7, 1> predState_;
    Eigen::Matrix<float, 7, 1> estState_;
    Eigen::MatrixXd pred_Var_;
    Eigen::MatrixXd est_Var_;
    float lambda_;
    Eigen::Matrix<float, 15, 1> W_mean_;
    Eigen::Matrix<float, 15, 1> W_cov_;


    // Noise stuff
    float angular_velocity_noise_var;
    std::default_random_engine gen;
    float angular_velocity_noise_stddev;
    std::normal_distribution<float> noise_dist;
    
};