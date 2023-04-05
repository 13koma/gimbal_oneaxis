#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO    22         // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO    21         // GPIO number for I2C master data
#define I2C_MASTER_NUM       I2C_NUM_0  // I2C port number for master dev
#define MPU6050_ADDR         0x68       // MPU6050 device address
#define CALIB_SAMPLES   1000

static const char *TAG = "gimbal-oneaxis";

static int16_t accel_x, accel_y, accel_z;
static int16_t gyro_x, gyro_y, gyro_z;
static int32_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
static int32_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
static int16_t accel_x_calibrated, accel_y_calibrated, accel_z_calibrated;
static int16_t gyro_x_calibrated, gyro_y_calibrated, gyro_z_calibrated;

esp_err_t i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;     // I2C master clock frequency
    conf.clk_flags = 0;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void mpu6050_init() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true);   // PWR_MGMT_1 register
    i2c_master_write_byte(cmd, 0x00, true);   // set to zero to wake up the sensor
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("MPU6050 initialization failed\n");
    }
}

void read_mpu6050_data() {
    uint8_t data[14];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);   // starting with register 0x3B (ACCEL_XOUT_H)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("Error reading MPU6050 data\n");
        return;
    }
    accel_x = (data[0] << 8) | data[1];
    accel_y = (data[2] << 8) | data[3];
    accel_z = (data[4] << 8) | data[5];
    gyro_x = (data[8] << 8) | data[9];
    gyro_y = (data[10] << 8) | data[11];
    gyro_z = (data[12] << 8) | data[13];
}

void calibrate_mpu6050() {
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        read_mpu6050_data();
        accel_x_offset += accel_x;
        accel_y_offset += accel_y;
        accel_z_offset += accel_z;
        gyro_x_offset += gyro_x;
        gyro_y_offset += gyro_y;
        gyro_z_offset += gyro_z;
        vTaskDelay(10 / portTICK_PERIOD_MS);   // wait for 10 ms
    }
    accel_x_offset /= CALIB_SAMPLES;
    accel_y_offset /= CALIB_SAMPLES;
    accel_z_offset /= CALIB_SAMPLES;
    gyro_x_offset /= CALIB_SAMPLES;
    gyro_y_offset /= CALIB_SAMPLES;
    gyro_z_offset /= CALIB_SAMPLES;
    // while (1) {
    //     read_mpu6050_data();
    //     accel_x_calibrated = accel_x - accel_x_offset;
    //     accel_y_calibrated = accel_y - accel_y_offset;
    //     accel_z_calibrated = accel_z - accel_z_offset;
    //     gyro_x_calibrated = gyro_x - gyro_x_offset;
    //     gyro_y_calibrated = gyro_y - gyro_y_offset;
    //     gyro_z_calibrated = gyro_z - gyro_z_offset;
    //     printf("Accelerometer: x=%d y=%d z=%d\n", accel_x_calibrated, accel_y_calibrated, accel_z_calibrated);
    //     printf("Gyroscope: x=%d y=%d z=%d\n", gyro_x_calibrated, gyro_y_calibrated, gyro_z_calibrated);
    //     vTaskDelay(100 / portTICK_PERIOD_MS);   // wait for 100 ms
    // }
}

void app_main() {
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        printf("I2C master initialization failed\n");
        return;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");
    mpu6050_init();
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    calibrate_mpu6050();
    while (1) {
        read_mpu6050_data();
        accel_x_calibrated = accel_x - accel_x_offset;
        accel_y_calibrated = accel_y - accel_y_offset;
        accel_z_calibrated = accel_z - accel_z_offset;
        gyro_x_calibrated = gyro_x - gyro_x_offset;
        gyro_y_calibrated = gyro_y - gyro_y_offset;
        gyro_z_calibrated = gyro_z - gyro_z_offset;
        printf("Accelerometer: x=%d y=%d z=%d\n", accel_x_calibrated, accel_y_calibrated, accel_z_calibrated);
        printf("Gyroscope: x=%d y=%d z=%d\n", gyro_x_calibrated, gyro_y_calibrated, gyro_z_calibrated);
        vTaskDelay(100 / portTICK_PERIOD_MS);   // wait for 100 ms
    }
    // while(1) {
    //     read_mpu6050_data();
    //     printf("Accelerometer: x=%d y=%d z=%d\n", accel_x_calibrated, accel_y_calibrated, accel_z_calibrated);
    //     printf("Gyroscope: x=%d y=%d z=%d\n", gyro_x_calibrated, gyro_y_calibrated, gyro_z_calibrated);
    //     vTaskDelay(100 / portTICK_PERIOD_MS);   // wait for 100 ms
    // }
}