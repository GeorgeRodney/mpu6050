#include "led.hpp"
#include "MPU6050.hpp"
#include "hardware/timer.h"
#include "OrientationFilter.hpp"
#include <stdio.h>
#include <cstdlib>

// int i2c_write_blocking (i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop)
#define GPIO_4 PICO_DEFAULT_I2C_SDA_PIN
#define GPIO_5 PICO_DEFAULT_I2C_SCL_PIN
#define ERROR_GPIO 15
#define OPERATE_GPIO 16
#define MPU_ADDRESS 0x68

double get_time()
{
    return time_us_64() / 1e6;
}

int main()
{
    stdio_init_all();

    OrientationFilter ukfFilter_;

    led operate_indicator_(OPERATE_GPIO);
    led error_indicator_(ERROR_GPIO);
    operate_indicator_.set(false);

    imu mpu6050_(GPIO_4, GPIO_5, i2c_default, MPU_ADDRESS);
    
    // Validate IMU
    uint8_t mpu_ID = mpu6050_.who_are_you();
    if(mpu_ID != MPU_ADDRESS)
    {
        error_indicator_.set(true);
        operate_indicator_.set(false);
        std::exit(EXIT_FAILURE);
    }

    // SETUP
    mpu6050_.calibrate_gyroscope();
    ukfFilter_.calcWeights();
    ukfFilter_.currentTime_ = get_time();

    while(true)
    {
        // What time is it
        operate_indicator_.set(true);
        ukfFilter_.lastTime_ = ukfFilter_.currentTime_;
        ukfFilter_.currentTime_ = get_time();
        ukfFilter_.dt_ = ukfFilter_.currentTime_ - ukfFilter_.lastTime_;

        // Predict State Forward DT
        ukfFilter_.predict(ukfFilter_.dt_);
        ukfFilter_.predictMeasurement();
        operate_indicator_.set(true);
        // Measure and innovate
        mpu6050_.read_gyroscope();
        ukfFilter_.calculateInnovation(mpu6050_.gyroData_);

        // Update State
        ukfFilter_.update();

        printf("%.4d,%.4d,%.4d,%.4d\n", ukfFilter_.est_State_(0), ukfFilter_.est_State_(1), ukfFilter_.est_State_(2), ukfFilter_.est_State_(3));

        operate_indicator_.set(false);
        sleep_ms(1000);
    }

}