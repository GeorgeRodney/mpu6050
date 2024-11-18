#include "led.hpp"
#include "MPU6050.hpp"
#include "hardware/timer.h"
#include "filterEKF.hpp"
// #include "filterUKF.hpp"
#include <stdio.h>
// #include "Wifi.hpp"

#define GPIO_4 PICO_DEFAULT_I2C_SDA_PIN
#define GPIO_5 PICO_DEFAULT_I2C_SCL_PIN
#define ERROR_GPIO 15
#define OPERATE_GPIO 16
#define MPU_ADDRESS 0x68

int main()
{
    stdio_init_all();

    filterEKF filter_;

    led operate_indicator_(OPERATE_GPIO);
    led error_indicator_(ERROR_GPIO);
    operate_indicator_.set(false);
    sleep_ms(10000);

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
    bool firstTimeIn = true;

    while(true)
    {
        // What time is it
        operate_indicator_.set(true);
        filter_.lastTime_ = filter_.currentTime_;
        filter_.currentTime_ = static_cast<float>(time_us_64() / 1000000.0);
        filter_.dt_ = filter_.currentTime_ - filter_.lastTime_;
        // filter_.dt_ = 0.025;

        // printf("last time   : %f\n", filter_.lastTime_);
        // printf("current time: %f\n", filter_.currentTime_);
        // printf("delta time  : %f\n", filter_.dt_);

        // Predict State Forward DT
        filter_.predict(filter_.dt_);
        operate_indicator_.set(true);
        // Measure and innovate
        mpu6050_.read_gyroscope();
        filter_.calculateInnovation(mpu6050_.gyroData_);

        // Update State
        filter_.update();

        printf("%.4f,%.4f,%.4f,%.4f\n", filter_.estState_(0), filter_.estState_(1), filter_.estState_(2), filter_.estState_(3));

        operate_indicator_.set(false);
        sleep_ms(32);
    }

}