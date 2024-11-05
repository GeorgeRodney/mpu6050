#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#define STATE_SIZE 7

class OrientationFilter
{
    public:

    OrientationFilter();

    ~OrientationFilter();

    void predict(double dt);

    void update(double measNoise);

    void calcWeights();

    Eigen::Matrix<double, 7, 1> predState_;
    Eigen::Matrix<double, 7, 1> estState_;
    Eigen::MatrixXd pred_Var_;
    Eigen::MatrixXd est_Var_;
    double lambda_;
    Eigen::MatrixXd W_mean_;
    Eigen::MatrixXd W_cov_;


    // Noise stuff
    double angular_velocity_noise_var;
    std::default_random_engine gen;
    double angular_velocity_noise_stddev;
    std::normal_distribution<double> noise_dist;
    
};