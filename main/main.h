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
#define MAX30102_FIFO_BURST_SIZE 1 // 16 Samples
#define BUFFER_SIZE 100  // Store 100 samples for BPM calculation
#define HB_THRESHOLD 2500
#define FILTER_ALPHA 0.3 // Adjust for responsiveness (0.1–0.5)
#define BANDPASS_BETA 0.05