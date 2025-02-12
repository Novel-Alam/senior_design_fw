#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"  // Updated as per deprecation warning
#include "driver/i2c_master.h"
#include "mpu6050.h"

// Define constants
#define X_LSB 0
#define X_MSB 1
#define Y_LSB 2
#define Y_MSB 3
#define Z_LSB 4
#define Z_MSB 5

#define GYRO_SCALING_FACTOR 131 // For ±250dps (FS_SEL=0)
#define ACCEL_SCALING_FACTOR 16384.0 // For ±2g (AFS_SEL=0)

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
i2c_device_config_t MPU_6050_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,  // Use 7-bit addressing
    .device_address = 0x68,                // MPU-6050 default address
    .scl_speed_hz = 100000,                // Set clock speed to 100 kHz
};

// Configure the MAX30102 device on the I2C bus
i2c_device_config_t MAX30102_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,  // Use 7-bit addressing
    .device_address = 0x57,                // MAX30102 default address
    .scl_speed_hz = 100000,                // Set clock speed to 100 kHz
};

// Handle for the MPU-6050 device
i2c_master_dev_handle_t MPU_6050_dev_handle;

i2c_master_dev_handle_t MAX30102_dev_handle;

void initialize_mpu6050() {
    const uint8_t PWR_MGMT_1_REG[1] = {0x6B};  // Power management register address

    // Transmit the register address to read its contents
    if (i2c_master_transmit(MPU_6050_dev_handle, PWR_MGMT_1_REG, 1, 100) != ESP_OK) {
        printf("Failed to transmit on i2c\n");
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay to allow operation completion

    uint8_t PWR_MGMT_1_REG_CONTENTS[1] = {0}; 
    // Read the contents of the power management register
    if (i2c_master_receive(MPU_6050_dev_handle, PWR_MGMT_1_REG_CONTENTS, 1, -1) == ESP_OK) {
        printf("PWR_Management_1: %x\n", PWR_MGMT_1_REG_CONTENTS[0]);
    }

    // Clear the sleep bit (bit 6) to wake up the MPU-6050
    uint8_t SET_NO_SLEEP_COMMAND[2] = {PWR_MGMT_1_REG[0], PWR_MGMT_1_REG_CONTENTS[0] & 0b10111111};
    if (i2c_master_transmit(MPU_6050_dev_handle, SET_NO_SLEEP_COMMAND, 2, 100) != ESP_OK) {
        printf("Error in turning on MPU6050\n");
    }

    // Recheck the power management register to verify sleep mode is disabled
    if (i2c_master_receive(MPU_6050_dev_handle, PWR_MGMT_1_REG_CONTENTS, 1, -1) == ESP_OK) {
        printf("PWR_Management_1: %x\n", PWR_MGMT_1_REG_CONTENTS[0]);
    }

    const uint8_t regAdr[1] = {0x75};  // WHO_AM_I register address (device ID)
    uint8_t byteRead[1];

    // Read WHO_AM_I register to verify device identity (expected value: 0x68)
    if (i2c_master_transmit_receive(MPU_6050_dev_handle, regAdr, 1, byteRead, 1, 100) != ESP_OK) {
        printf("Failed to transmit on i2c\n");
    }
    printf("Address %u\n", byteRead[0]);
}

void read_gyroscope_data(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
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
        if (i2c_master_transmit_receive(MPU_6050_dev_handle, &gyro_reg_addresses[i], 1, &gyroscopeData[i], 1, 100) != ESP_OK) {
            printf("Failed to transmit on i2c\n");
        }
    }
    
    *gyro_x = (int16_t)((gyroscopeData[1] << 8) | gyroscopeData[0]);
    *gyro_y = (int16_t)((gyroscopeData[3] << 8) | gyroscopeData[2]);
    *gyro_z = (int16_t)((gyroscopeData[5] << 8) | gyroscopeData[4]);
}

void read_accelerometer_data(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z) {
    uint8_t accelData[6]; // Stores X/Y/Z high/low bytes

    uint8_t accel_reg_addresses[6] = {
        MPU6050_RA_ACCEL_XOUT_L, MPU6050_RA_ACCEL_XOUT_H,
        MPU6050_RA_ACCEL_YOUT_L, MPU6050_RA_ACCEL_YOUT_H,
        MPU6050_RA_ACCEL_ZOUT_L, MPU6050_RA_ACCEL_ZOUT_H
    };
    
    // Read all 6 accelerometer registers
    for (int i = 0; i < 6; i++) {
        i2c_master_transmit_receive(MPU_6050_dev_handle, &accel_reg_addresses[i], 1, &accelData[i], 1, 100);
    }
    
    *accel_x = (int16_t)((accelData[1] << 8) | accelData[0]);
    *accel_y = (int16_t)((accelData[3] << 8) | accelData[2]);
    *accel_z = (int16_t)((accelData[5] << 8) | accelData[4]);
}

void initialize_MAXIM30102() {
    uint8_t data[2];

    // Reset the device
    data[0] = 0x09;  // MODE_CONFIG register
    data[1] = 0x40;  // RESET command
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    // Configure FIFO (Sample averaging = 4, FIFO rollover enabled, almost full = 17 samples)
    data[0] = 0x08;  // FIFO_CONFIG register
    data[1] = 0x4F;  // Configuration value
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    // Set SpO2 mode (Heart rate mode)
    data[0] = 0x09;  // MODE_CONFIG register
    data[1] = 0x03;  // Heartbeat mode (bits [2:0] = 011)
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    // Set LED pulse amplitudes
    data[0] = 0x0C;  // LED1_PA register (Red LED)
    data[1] = 0x24;  // Red LED current = ~36 mA
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    data[0] = 0x0D;  // LED2_PA register (IR LED)
    data[1] = 0x24;  // IR LED current = ~36 mA
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));
}


void app_main(void) {
    
    // Initialize the I2C master bus and add the MPU-6050 device to it
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &MPU_6050_dev_cfg, &MPU_6050_dev_handle));
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &MAX30102_dev_cfg, &MAX30102_dev_handle));
    
    // initialize_mpu6050();
    initialize_MAXIM30102();
    
    for (;;) {
        int16_t accel_x_raw, accel_y_raw, accel_z_raw;
        int16_t gyroscope_x_raw, gyroscope_y_raw, gyroscope_z_raw;
        
        // read_accelerometer_data(&accel_x_raw, &accel_y_raw, &accel_z_raw);
        // read_gyroscope_data(&gyroscope_x_raw, &gyroscope_y_raw, &gyroscope_z_raw);
        
        // Convert to g-force
        float accel_x = accel_x_raw / ACCEL_SCALING_FACTOR;
        float accel_y = accel_y_raw / ACCEL_SCALING_FACTOR;
        float accel_z = accel_z_raw / ACCEL_SCALING_FACTOR;
        
        // Print accelerometer data
        // printf("Acceleration (m/s²): X=%.2f, Y=%.2f, Z=%.2f\n", accel_x, accel_y, accel_z);
        
        // Use correct scaling factor based on FS_SEL
        float gyro_x = gyroscope_x_raw / GYRO_SCALING_FACTOR;
        float gyro_y = gyroscope_y_raw / GYRO_SCALING_FACTOR;
        float gyro_z = gyroscope_z_raw / GYRO_SCALING_FACTOR;
        
        // Apply calibration offsets
        gyro_x += GYRO_X_CALIBRATION;
        gyro_y += GYRO_Y_CALIBRATION;
        gyro_z += GYRO_Z_CALIBRATION;
        
        // Print gyroscope data
        // printf("Gyroscope data (dps): X = %.2f, Y = %.2f, Z = %.2f\n", gyro_x, gyro_y, gyro_z);
        
        // Step 1: Write the FIFO_DATA register address (0x07) to the MAX30102
        uint8_t FIFO_REG_ADDR = 0x07;
        if (i2c_master_transmit(MAX30102_dev_handle, &FIFO_REG_ADDR, 1, 100) != ESP_OK) {
            printf("Failed to write FIFO register address\n");
            return;
        }

        // Step 2: Perform a repeated start condition and read the FIFO data
        uint8_t FIFO_DATA[6];  // Buffer to store the FIFO data (6 bytes for one sample)
        if (i2c_master_receive(MAX30102_dev_handle, FIFO_DATA, 6, 100) != ESP_OK) {
            printf("Failed to read FIFO data\n");
            return;
        }

        // Step 3: Process the FIFO data
        // The FIFO data contains 3 bytes for the Red channel and 3 bytes for the IR channel
        uint32_t red_value = (FIFO_DATA[0] << 16) | (FIFO_DATA[1] << 8) | FIFO_DATA[2];
        uint32_t ir_value = (FIFO_DATA[3] << 16) | (FIFO_DATA[4] << 8) | FIFO_DATA[5];

        printf("Red Value: %lu, IR Value: %lu\n", red_value, ir_value);

        
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Short delay before next iteration
    }
}
