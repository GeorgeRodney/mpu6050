#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#define STATE_SIZE 7
#define MEAS_SIZE 3

class OrientationFilter
{
    public:

    OrientationFilter();

    ~OrientationFilter();

    void predict(double dt);
    void predictMeasurement();
    void calculateInnovation(const float gyroMeasIn[3]);
    void update();
    void calcWeights();

    Eigen::VectorXd pred_State_;
    Eigen::VectorXd est_State_;
    Eigen::VectorXd innovation_;
    Eigen::MatrixXd pred_Var_;
    Eigen::MatrixXd est_Var_;
    Eigen::MatrixXd innovation_Var_;       
    Eigen::VectorXd pred_Meas_;
    Eigen::VectorXd est_Meas_;
    double lambda_;
    Eigen::MatrixXd W_mean_;
    Eigen::MatrixXd W_cov_;
    Eigen::MatrixXd state_meas_cov_;
    Eigen::MatrixXd K;
    Eigen::MatrixXd predicted_sigma_;
    Eigen::MatrixXd measurement_sigma_;
    Eigen::MatrixXd measurement_Var_;


    double currentTime_;
    double lastTime_;
    double dt_;

    // Noise stuff
    double angular_velocity_noise_var;
    std::default_random_engine gen;
    double angular_velocity_noise_stddev;
    Eigen::MatrixXd measurement_noise_matrix_;
    std::normal_distribution<double> noise_dist;
    
};