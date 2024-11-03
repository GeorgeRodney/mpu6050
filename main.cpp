#include "led.hpp"
#include "MPU6050.hpp"
#include "hardware/timer.h"
#include <stdio.h>
#include <cstdlib>

// int i2c_write_blocking (i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop)
#define GPIO_4 PICO_DEFAULT_I2C_SDA_PIN
#define GPIO_5 PICO_DEFAULT_I2C_SCL_PIN
#define ERROR_GPIO 15
#define OPERATE_GPIO 16
#define MPU_ADDRESS 0x68

int main()
{
    stdio_init_all();

    led operate_indicator_(OPERATE_GPIO);
    led error_indicator_(ERROR_GPIO);
    operate_indicator_.set(true);

    imu mpu6050_(GPIO_4, GPIO_5, i2c_default, MPU_ADDRESS);
    sleep_ms(10);

    // Validate IMU
    uint8_t mpu_ID = mpu6050_.who_are_you();
    if(mpu_ID != MPU_ADDRESS)
    {
        error_indicator_.set(true);
        operate_indicator_.set(false);
        std::exit(EXIT_FAILURE);
    }

    // CALIBRATIING
    mpu6050_.calibrate_gyroscope();

    while(1)
    {
        operate_indicator_.set(false);
        sleep_ms(500);
        operate_indicator_.set(true);
        sleep_ms(500);

        printf("GYRO DATA X: %f\n", mpu6050_.gyroData_[0]);
        printf("GYRO DATA Y: %f\n", mpu6050_.gyroData_[1]);
        printf("GYRO DATA Z: %f\n", mpu6050_.gyroData_[2]);
        printf("\n");
        printf("ACCEL DATA X: %f\n", mpu6050_.accelData_[0]);
        printf("ACCEL DATA Y: %f\n", mpu6050_.accelData_[1]);
        printf("ACCEL DATA Z: %f\n", mpu6050_.accelData_[2]);
        printf("\n\n");

        sleep_ms(100);
    }

}