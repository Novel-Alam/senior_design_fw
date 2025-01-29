// #include "mpu6050.h"
// #include "i2c_development.h"
// void initialize(MPU6050_t *MPU6050, uint8_t address, void *wireObj) {
//     if (MPU6050 == NULL){
//         MPU6050 = (MPU6050_t*)(sizeof(MPU6050_t));
//     }
//     MPU6050->devAddr = address;
//     MPU6050->wireObj = wireObj;
//     setClockSource(MPU6050_CLOCK_PLL_XGYRO);
//     setFullScaleGyroRange(MPU6050_GYRO_FS_250);
//     setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//     setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
// }

// void setClockSource(uint8_t source){

// }

// void setFullScaleGyroRange(uint8_t range){

// }

// void setFullScaleAccelRange(uint8_t range){

// }

// void setSleepEnabled(bool enabled){

// }