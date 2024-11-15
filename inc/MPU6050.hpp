#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <stdio.h>
#include "hardware/timer.h"
#include <Eigen/Dense>
#include <cmath>

class imu
{   
public:

    imu(int SDA, int SCL, i2c_inst_t* I2C_PORT, uint8_t I2C_ADDRESS)
    :   sdaPin_(SDA),
        sclPin_(SCL),
        i2cPort_(I2C_PORT),
        i2cAddress_(I2C_ADDRESS),
        gyroOffset_{0.0f, 0.0f, 0.0f},
        accelData_{0.0f, 0.0f, 0.0f},
        gyroData_{0.0f, 0.0f, 0.0f},
        MEAS_PERIOD(100),
        WHO_AM_I(0x75),
        ACCEL_REG(0x3B),
        GYRO_REG(0x43)
    {
        // This line will call static_timer_callback() and will produce a gyro reading at the MEAS_PERIOD
        // add_repeating_timer_ms(MEAS_PERIOD, static_timer_callback, this, &timer_);

        i2c_init(i2cPort_, 400*1000);
        gpio_set_function(sdaPin_, GPIO_FUNC_I2C);
        gpio_set_function(sclPin_, GPIO_FUNC_I2C);
        bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

        uint8_t initBuff[] = {0x6B, 0x00};
        i2c_write_blocking(i2c_default, i2cAddress_, initBuff, 2, false);
        sleep_ms(1000);
    }

    ~imu()
    {
        i2c_deinit(i2cPort_);
    }

    uint8_t who_are_you()
    {
        uint8_t outByte;
        i2c_write_blocking(i2cPort_, i2cAddress_, &WHO_AM_I, 1, true);
        i2c_read_blocking(i2cPort_, i2cAddress_, &outByte, 1, false);

        return outByte;
    }

    uint8_t read_address(uint8_t read_address)
    {  
        uint8_t outByte;
        i2c_write_blocking(i2cPort_, i2cAddress_, &read_address, 1, true);
        i2c_read_blocking(i2cPort_, i2cAddress_, &outByte, 1, false);

        return outByte;
    }

    void read_accelerometer()
    {   
        uint8_t accel_data[6];
        int16_t temp[3];
        i2c_write_blocking(i2cPort_, i2cAddress_, &ACCEL_REG, 1, true);
        i2c_read_blocking(i2cPort_, i2cAddress_, accel_data, 6, false);

        for (int i = 0; i < 3; i++)
        {
            temp[i] = (accel_data[i * 2] << 8 | accel_data[(i * 2) + 1]);
        }

        for (int i = 0; i < 3; ++i)
        {
            accelData_[i] = -(float)temp[i] / 16384;
        }

    }

    void read_gyroscope()
    {
        uint8_t gyro_data[6];
        int16_t temp[3];
        i2c_write_blocking(i2cPort_, i2cAddress_, &GYRO_REG, 1, true);
        i2c_read_blocking(i2cPort_, i2cAddress_, gyro_data, 6, false);

        for (int i = 0; i < 3; i++)
        {
            temp[i] = (gyro_data[i * 2] << 8 | gyro_data[(i * 2) + 1]);
        }

        for (int i = 0; i < 3; ++i)
        {
            gyroData_[i] = -(float)temp[i] / 131;
            gyroData_[i] = gyroData_[i] - gyroOffset_[i];
            gyroData_[i] = gyroData_[i] * (M_PI / 180.0); // Convert to radians / sec
        }
    }

    // THIS FUNCTION REQUIRES THAT THE IMU BE COMPLETELY
    // MOTIONLESS DURING CALIBRATION
    void calibrate_gyroscope(void)
    {
        uint16_t samples = 100;

        float sum[3] = {0.0, 0.0, 0.0};
        float output[3] = {0.0, 0.0, 0.0};
        uint8_t gyro_data[6];
        int16_t temp[3];

        for (int calSamples = 0; calSamples < samples; ++calSamples)
        {
            // SAMPLE THE GYRO
            i2c_write_blocking(i2cPort_, i2cAddress_, &GYRO_REG, 1, true);
            i2c_read_blocking(i2cPort_, i2cAddress_, gyro_data, 6, false);
            for (int i = 0; i < 3; i++)
            {
                temp[i] = (gyro_data[i * 2] << 8 | gyro_data[(i * 2) + 1]);
            }
            for (int i = 0; i < 3; ++i)
            {
                output[i] = -(float)temp[i] / 131;
            }

            // RECORD THE VALUE
            for (int sumIdx = 0; sumIdx < 3; ++sumIdx)
            {
                sum[sumIdx] = sum[sumIdx] + output[sumIdx];
            }
        }

        // Take the Arithmetic Average
        for (int idx = 0; idx < 3; ++idx)
        {
            gyroOffset_[idx] = (float)sum[idx] / samples;
            // printf("gyroOffset: %f\n", gyroOffset_[idx]);
        }
    }

    static bool static_timer_callback(repeating_timer_t *rt)
    {
        imu* self = (imu*)rt->user_data;
        self->timer_callback();
        return true;
    }

    void timer_callback(void)
    {
        read_accelerometer();
        read_gyroscope();
        uint64_t read_time = to_ms_since_boot(get_absolute_time());

        measMessage_.block<3,1>(0,0) << accelData_[0], accelData_[1], accelData_[2];
        measMessage_.block<3,1>(3,0) << gyroData_[0], gyroData_[1], gyroData_[2];
        measMessage_.block<1,1>(6,0) << read_time;

        // printf("Time: %d\n", read_time);
        // printf("Accel: X: %f, Y: %f, Z: %f\n", accelData_[0], accelData_[1], accelData_[2]);
        // printf("Gyro: X: %f, Y: %f, Z: %f\n", gyroData_[0], gyroData_[1], gyroData_[2]);
        // printf("measMessage: accelX: %f, accelY: %f, accelZ: %f, gyroX: %f, gyroY: %f, gyroZ: %f, time: %d\n", measMessage_[0], measMessage_[1], measMessage_[2], measMessage_[3], measMessage_[4], measMessage_[5], measMessage_[6]);
        // printf("\n\n");
    }

    int sdaPin_;
    int sclPin_;
    i2c_inst_t* i2cPort_;
    uint8_t i2cAddress_;
    float gyroOffset_[3];
    float accelData_[3];
    float gyroData_[3];
    repeating_timer_t timer_;
    Eigen::Matrix<float, 7, 1> measMessage_;
    
    // Addresses
    const uint16_t MEAS_PERIOD;
    const uint8_t WHO_AM_I;
    const uint8_t ACCEL_REG;
    const uint8_t GYRO_REG;

};