#ifdef COMMON_H
#define COMMON_H

#include "mpu6050.h"
#include "esp_system.h"
// I2C Data
#define I2C_MASTER_SCL_IO 7  // SCL pin (set based on your configuration)
#define I2C_MASTER_SDA_IO 6  // SDA pin (set based on your configuration)

// Defines for XYZ indices
#define X_LSB 0
#define X_MSB 1
#define Y_LSB 2
#define Y_MSB 3
#define Z_LSB 4
#define Z_MSB 5


#endif