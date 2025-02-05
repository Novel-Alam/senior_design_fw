// #include "common.h"

// esp_err_t get_gyro_data(){
//     for (int i = X_LSB; i <= Z_MSB; i++;){ //

//     }
// }
//         // // Read low byte of X-axis gyroscope data
//         // if (i2c_master_transmit_receive(dev_handle, MPU6050_RA_GYRO_XOUT_L, 1, &gyroscopeData[X_LSB], 1, 100) != ESP_OK) {
//         //     printf("Failed to transmit on i2c\n");
//         // } 

//         // // Read high byte of X-axis gyroscope data
//         // if (i2c_master_transmit_receive(dev_handle, MPU6050_RA_GYRO_XOUT_H, 1, &gyroscopeData[X_MSB], 1, 100) != ESP_OK) {
//         //     printf("Failed to transmit on i2c\n");
//         // } 

//         // // Read low byte of Y-axis gyroscope data
//         // if (i2c_master_transmit_receive(dev_handle, MPU6050_RA_GYRO_YOUT_L, 1, &gyroscopeData[Y_LSB], 1, 100) != ESP_OK) {
//         //     printf("Failed to transmit on i2c\n");
//         // } 

//         // // Read high byte of Y-axis gyroscope data
//         // if (i2c_master_transmit_receive(dev_handle, MPU6050_RA_GYRO_YOUT_H, 1, &gyroscopeData[Y_MSB], 1, 100) != ESP_OK) {
//         //     printf("Failed to transmit on i2c\n");
//         // } 

//         // // Read low byte of Z-axis gyroscope data
//         // if (i2c_master_transmit_receive(dev_handle, MPU6050_RA_GYRO_ZOUT_L, 1, &gyroscopeData[Z_LSB], 1, 100) != ESP_OK) {
//         //     printf("Failed to transmit on i2c\n");
//         // } 

//         // // Read high byte of Z-axis gyroscope data
//         // if (i2c_master_transmit_receive(dev_handle, MPU6050_RA_GYRO_ZOUT_H, 1, &gyroscopeData[Z_MSB], 1, 100) != ESP_OK) {
//         //     printf("Failed to transmit on i2c\n");
//         // } 