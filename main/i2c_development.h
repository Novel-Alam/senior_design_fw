// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "spi_flash_mmap.h"  // Updated as per deprecation warning
// #include "driver/i2c_master.h"

// typedef struct {
//     static uint16_t readTimeout;
// } i2c_obj;

// static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=I2Cdev::readTimeout, void *wireObj=0);


// /** Read multiple bytes from an 8-bit device register.
//  * @param devAddr I2C slave device address
//  * @param regAddr First register regAddr to read from
//  * @param length Number of bytes to read
//  * @param data Buffer to store read data in
//  * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
//  * @return Status of read operation
//  */

// int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout, void *wireObj) {
// 	i2c_cmd_handle_t cmd;
// 	SelectRegister(devAddr, regAddr);

// 	cmd = i2c_cmd_link_create();
// 	ESP_ERROR_CHECK(i2c_master_start(cmd));
// 	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, 1));

// 	if(length>1)
// 		ESP_ERROR_CHECK(i2c_master_read(cmd, data, length-1, I2C_MASTER_ACK));

// 	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+length-1, I2C_MASTER_NACK));

// 	ESP_ERROR_CHECK(i2c_master_stop(cmd));
// 	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
// 	i2c_cmd_link_delete(cmd);

// 	return length;
// }

// /** Write single byte to an 8-bit device register.
//  * @param devAddr I2C slave device address
//  * @param regAddr Register address to write to
//  * @param length Number of bytes to write
//  * @param data Array of bytes to write
//  * @return Status of operation (true = success)
//  */
// uint8_t writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, void *wireObj){
	
//     i2c_cmd_handle_t cmd; //I2C Commands Queue, master by default

// 	cmd = i2c_cmd_link_create(); //Create/initialize cmd queue struct
// 	ESP_ERROR_CHECK(i2c_master_start(cmd)); //Send start signal to queue
// 	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
// 	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
// 	ESP_ERROR_CHECK(i2c_master_write(cmd, data, length-1, 0));
// 	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length-1], 1));
// 	ESP_ERROR_CHECK(i2c_master_stop(cmd));
// 	ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
// 	i2c_cmd_link_delete(cmd);   
// 	return true;
// }