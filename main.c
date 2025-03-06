#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"  // Updated as per deprecation warning
#include "driver/i2c_master.h"
#include "mpu6050.h"
#include "esp_systick_etm.h"
#include "esp_timer.h"
// #include "esp_systick_etm.h"

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

#define MAX30102_FIFO_SAMPLE_SIZE 6 // 6 Bytes Per "Sample" (Red[0:2]_IR[3:5])
#define MAX30102_FIFO_BURST_SIZE 16 // 16 Samples
#define BUFFER_SIZE 100  // Store 100 samples for BPM calculation
#define HB_THRESHOLD 2500
#define FILTER_ALPHA 0.5 // Adjust for responsiveness (0.1–0.5)


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


uint32_t ir_buffer[BUFFER_SIZE];
uint8_t buffer_index = 0;
uint32_t burst_ir_average;
float filtered_ir = 0;
float signal_min = 1e6, signal_max = 0;
float threshold = 0;


// Apply a simple IIR bandpass filter
void update_filtered_ir(uint32_t raw_ir) {
    static float last_value = 0;
    filtered_ir = FILTER_ALPHA * raw_ir + (1 - FILTER_ALPHA) * last_value;
    last_value = filtered_ir;
}

void update_thresholds(float filtered_ir) {
    // Track min/max with exponential decay
    signal_min = 0.99 * signal_min + 0.01 * filtered_ir;
    signal_max = 0.99 * signal_max + 0.01 * filtered_ir;
    threshold = signal_min + (signal_max - signal_min) * 0.5; // Midpoint
}

void calculate_bpm(float filtered_ir, uint32_t current_time) {
    static uint32_t last_beat_time = 0;
    static float beat_avg = 70.0; // Initialize with a realistic BPM
    static float last_value = 0;
    float bpm = 0.0;
    printf("Threshold: %.1f, Filtered IR: %.1f\n", filtered_ir, threshold);
    // Detect rising edge crossing the threshold
    if (filtered_ir > threshold && last_value <= threshold) {
        if (last_beat_time != 0) {
            uint32_t beat_interval = current_time - last_beat_time;
            if (beat_interval > 400) { // Refractory period = 400ms (max 150 BPM)
                bpm = 60000.0 / beat_interval;
                printf("bpm: %.1f\n\n\n\n\n\n\n", bpm);
                beat_avg = 0.7 * beat_avg + 0.3 * bpm; // Smoothing
            }
        }
        last_beat_time = current_time;
    }
    last_value = filtered_ir;

    printf("bpm avg: %.1f\n", beat_avg);
}



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
    burst_ir_average = 0;

    // Reset the device
    data[0] = 0x09;  // MODE_CONFIG register
    data[1] = 0x40;  // RESET command
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    // Configure FIFO (Sample averaging = 4, FIFO rollover enabled, almost full = 17 samples)
    data[0] = 0x08;  // FIFO_CONFIG register
    // data[1] = 0x4F;  // Configuration value
    data[1] = 0x30 | 0x08;  // Sample rate = 400 Hz (0x60), FIFO almost full = 8 (0x08)
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    // Set SpO2 mode (Heart rate mode)
    data[0] = 0x09;  // MODE_CONFIG register
    //TODO: Spo2 Register Configuration, not MODE_CONFIG register
    data[1] = 0x03 | (0x03 << 3); // Mode = SpO2 (0x03), Pulse Width = 411µs (0x03 << 3), ADC range = 4096nA (0x03)
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    // Set LED pulse amplitudes
    data[0] = 0x0C;  // LED1_PA register (Red LED)
    data[1] = 0x1E;  // Red LED current = ~30 mA
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));

    data[0] = 0x0D;  // LED2_PA register (IR LED)
    data[1] = 0x24;  // IR LED current = ~36 mA
    ESP_ERROR_CHECK(i2c_master_transmit(MAX30102_dev_handle, data, 2, 100));
}





typedef enum {
    WAITING_FOR_FIRST_PEAK,
    WAITING_FOR_SECOND_PEAK
} BPMState;

int calculate_bpm2(uint32_t raw_ir) {
    // Static variables to maintain state between function calls.
    static BPMState state = WAITING_FOR_FIRST_PEAK;
    static uint32_t first_peak_time = 0;
    static uint32_t rolling_max = 0;
    static uint32_t rolling_max_time = 0;
    
    // Get current time in ms using esp_timer_get_time (which returns time in µs)
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // Update the rolling maximum value if the current raw_ir is higher.
    if (raw_ir > rolling_max) {
        rolling_max = raw_ir;
        rolling_max_time = current_time;
    }
    
    // Check if the rolling maximum has been maintained for at least 400ms.
    if ((current_time - rolling_max_time) >= 400) {
        // Committed peak detected.
        if (state == WAITING_FOR_FIRST_PEAK) {
            // Save timestamp for the first peak and change state.
            first_peak_time = rolling_max_time;
            state = WAITING_FOR_SECOND_PEAK;
        } else if (state == WAITING_FOR_SECOND_PEAK) {
            // Calculate period between peaks.
            uint32_t period = rolling_max_time - first_peak_time;
            // Reset state for the next measurement cycle.
            state = WAITING_FOR_FIRST_PEAK;
            // Reset rolling max for new measurement.
            rolling_max = 0;
            // Convert period (ms) to BPM: BPM = 60000 / period.
            return (int)(60000.0 / period);
        }
        // Reset the rolling maximum for subsequent measurements.
        rolling_max = 0;
    }
    
    // Not enough data (peak-to-peak interval not available) to compute BPM.
    return -1;
}




void app_main(void) {
    
    // Initialize the I2C master bus and add the MPU-6050 device to it
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &MPU_6050_dev_cfg, &MPU_6050_dev_handle));
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &MAX30102_dev_cfg, &MAX30102_dev_handle));
    
    initialize_mpu6050();
    initialize_MAXIM30102();
    uint32_t rolling_ir_average  = 0;
    
    for (;;) {
        int16_t accel_x_raw, accel_y_raw, accel_z_raw;
        int16_t gyroscope_x_raw, gyroscope_y_raw, gyroscope_z_raw;
        
        read_accelerometer_data(&accel_x_raw, &accel_y_raw, &accel_z_raw);
        read_gyroscope_data(&gyroscope_x_raw, &gyroscope_y_raw, &gyroscope_z_raw);
        
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
        uint8_t FIFO_DATA_LEN = MAX30102_FIFO_BURST_SIZE * MAX30102_FIFO_SAMPLE_SIZE; //Burst size * Sample Size, bytes
        uint8_t FIFO_DATA[FIFO_DATA_LEN];  // Buffer to store the FIFO data (6 bytes for one sample)
        if (i2c_master_receive(MAX30102_dev_handle, FIFO_DATA, FIFO_DATA_LEN, 100) != ESP_OK) {
            printf("Failed to read FIFO data\n");
            return;
        }

        // Step 3: Process the FIFO data
        // The FIFO data contains 3 bytes for the Red channel and 3 bytes for the IR channel
        uint32_t red_value;
        uint32_t ir_value;

        uint8_t *sample;
        burst_ir_average = 0;
        uint32_t current_time = 0;  // ms
        for(int i = 0; i < FIFO_DATA_LEN; i+=6){
            sample = &FIFO_DATA[i];
            red_value = (sample[0] << 16) | (sample[1] << 8) | sample[2];
            ir_value = (sample[3] << 16) | (sample[4] << 8) | sample[5];
            // printf("Red Value: %lu, IR Value: %lu\n", red_value, ir_value);
            
            // Filter and update thresholds
            // update_filtered_ir(ir_value);
        }
        

        bpm = calculate_bpm2(ir_value);
        if (bpm != -1) {
            printf("Calculated BPM: %d\n", bpm);
        }


            // update_thresholds(filtered_ir);
            
            // current_time = esp_timer_get_time() / 1000;
            // // printf("Filtered IR: %.1f\n", filtered_ir);
            // // Detect beats
            // calculate_bpm(filtered_ir, current_time);
        // }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Short delay before next iteration
    }
}
