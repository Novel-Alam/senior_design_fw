#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"  // Updated as per deprecation warning
#include "driver/i2c_master.h"
#include "mpu6050.h"

#define X_LSB 0
#define X_MSB 1
#define Y_LSB 2
#define Y_MSB 3
#define Z_LSB 4
#define Z_MSB 5

#define GYRO_SCALING_FACTOR 131 // For ±250dps (FS_SEL=0)
// Define I2C port and GPIO pins for SDA and SCL
#define TEST_I2C_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO 7  // SCL pin (set based on your configuration)
#define I2C_MASTER_SDA_IO 6  // SDA pin (set based on your configuration)

#define GYRO_X_CALIBRATION 1.0
#define GYRO_Y_CALIBRATION -1.0
#define GYRO_Z_CALIBRATION 0.0

// Configure the I2C master bus
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,  // Use default clock source
    .i2c_port = TEST_I2C_PORT,          // Specify the I2C port
    .scl_io_num = I2C_MASTER_SCL_IO,    // GPIO for SCL
    .sda_io_num = I2C_MASTER_SDA_IO,    // GPIO for SDA
    .glitch_ignore_cnt = 7,             // Ignore glitches
    .flags.enable_internal_pullup = false,  // Disable internal pull-ups
};

// Handle for the I2C bus
i2c_master_bus_handle_t bus_handle;

// Configure the MPU-6050 device on the I2C bus
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,  // Use 7-bit addressing
    .device_address = 0x68,                // MPU-6050 default address
    .scl_speed_hz = 100000,                // Set clock speed to 100 kHz
};

// Handle for the MPU-6050 device
i2c_master_dev_handle_t dev_handle;

void app_main(void)
{
    // Initialize the I2C master bus and add the MPU-6050 device to it
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    const uint8_t PWR_MGMT_1_REG[1] = {0x6B};  // Power management register address

    // Transmit the register address to read its contents
    if (i2c_master_transmit(dev_handle, PWR_MGMT_1_REG, 1, 100) != ESP_OK) {
        printf("Failed to transmit on i2c\n");
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay to allow operation completion

    uint8_t PWR_MGMT_1_REG_CONTENTS[1] = {0}; 
    // Read the contents of the power management register
    if (i2c_master_receive(dev_handle, PWR_MGMT_1_REG_CONTENTS, 1, -1) == ESP_OK) {
        printf("PWR_Management_1: %x\n", PWR_MGMT_1_REG_CONTENTS[0]);
    }

    // Clear the sleep bit (bit 6) to wake up the MPU-6050
    uint8_t SET_NO_SLEEP_COMMAND[2] = {PWR_MGMT_1_REG[0], PWR_MGMT_1_REG_CONTENTS[0] & 0b10111111};
    if (i2c_master_transmit(dev_handle, SET_NO_SLEEP_COMMAND, 2, 100) != ESP_OK) {
        printf("Error in turning on MPU6050\n");
    }

    // Recheck the power management register to verify sleep mode is disabled
    if (i2c_master_receive(dev_handle, PWR_MGMT_1_REG_CONTENTS, 1, -1) == ESP_OK) {
        printf("PWR_Management_1: %x\n", PWR_MGMT_1_REG_CONTENTS[0]);
    }

    const uint8_t regAdr[1] = {0x75};  // WHO_AM_I register address (device ID)
    uint8_t byteRead[1];

    // Read WHO_AM_I register to verify device identity (expected value: 0x68)
    if (i2c_master_transmit_receive(dev_handle, regAdr, 1, byteRead, 1, 100) != ESP_OK) {
        printf("Failed to transmit on i2c\n");
    }
    printf("Address %u\n", byteRead[0]);


    for (;;)
    {
        
        /* 2 byte to hold x_gyroscope_data */
        uint8_t gyroscopeData[6]; //x_lsb is 0, y_lsb is 2, z_lsb is 4


        // Define register addresses for gyroscope data
        uint8_t gyro_reg_addresses[6] = {
            MPU6050_RA_GYRO_XOUT_L,
            MPU6050_RA_GYRO_XOUT_H,
            MPU6050_RA_GYRO_YOUT_L,
            MPU6050_RA_GYRO_YOUT_H,
            MPU6050_RA_GYRO_ZOUT_L,
            MPU6050_RA_GYRO_ZOUT_H
        };

        // Read gyroscope data for all axes
        for (int i = 0; i < 6; i++) {
            if (i2c_master_transmit_receive(dev_handle, &gyro_reg_addresses[i], 1, &gyroscopeData[i], 1, 100) != ESP_OK) {
            printf("Failed to transmit on i2c\n");
            }
        }

        

        // Replace uint16_t with int16_t for signed data
        int16_t gyroscope_x_raw = (int16_t)((gyroscopeData[1] << 8) | gyroscopeData[0]);
        int16_t gyroscope_y_raw = (int16_t)((gyroscopeData[3] << 8) | gyroscopeData[2]);
        int16_t gyroscope_z_raw = (int16_t)((gyroscopeData[5] << 8) | gyroscopeData[4]);

        // Apply calibration offsets (see calibration steps below)
 

        // Use correct scaling factor based on FS_SEL
        float gyro_x = gyroscope_x_raw / GYRO_SCALING_FACTOR;
        float gyro_y = gyroscope_y_raw / GYRO_SCALING_FACTOR;
        float gyro_z = gyroscope_z_raw / GYRO_SCALING_FACTOR;

        /* Do offsetsfrom float*/
        gyro_x += GYRO_X_CALIBRATION;
        gyro_y += GYRO_Y_CALIBRATION;
        gyro_z += GYRO_Z_CALIBRATION;

        // Print gyroscope data
        printf("Gyroscope data (dps): X = %.2f, Y = %.2f, Z = %.2f\n", gyro_x, gyro_y, gyro_z);

        vTaskDelay(100 / portTICK_PERIOD_MS);  // Short delay before next iteration

    }
}
