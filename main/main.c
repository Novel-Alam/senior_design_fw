#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"  // Updated as per deprecation warning
#include "driver/i2c_master.h"

// Define I2C port and GPIO pins for SDA and SCL
#define TEST_I2C_PORT I2C_NUM_0
#define I2C_MASTER_SCL_IO 7  // SCL pin (set based on your configuration)
#define I2C_MASTER_SDA_IO 6  // SDA pin (set based on your configuration)

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

    for (;;)
    {
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

        const uint8_t regAdr1[1] = {0x43};  // GYRO_XOUT_H register address (high byte of X-axis gyroscope)
        
        // Read high byte of X-axis gyroscope data
        if (i2c_master_transmit_receive(dev_handle, regAdr1, 1, byteRead, 1, 100) != ESP_OK) {
            printf("Failed to transmit on i2c\n");
        } else {
            printf("Success\n");
        }
        printf("GYRO_XOUT_H: %d\n", byteRead[0]);

        const uint8_t regAdr2[1] = {0x44};  // GYRO_XOUT_L register address (low byte of X-axis gyroscope)

        // Read low byte of X-axis gyroscope data
        if (i2c_master_transmit_receive(dev_handle, regAdr2, 1, byteRead, 1, 100) != ESP_OK) {
            printf("Failed to transmit on i2c\n");
        } else {
            printf("Success\n");
        }
        printf("GYRO_XOUT_L: %d\n", byteRead[0]);

        vTaskDelay(10 / portTICK_PERIOD_MS);  // Short delay before next iteration
    }
}
