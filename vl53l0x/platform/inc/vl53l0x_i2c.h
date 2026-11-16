/**
 * VL53L0X I2C 硬件层 — MSPM0 DriverLib
 * 使用 MPU6050 I2C (I2C0, PA0=SDA, PA1=SCL) — shared with VL53L0X
 */
#ifndef __VL53L0X_I2C_H
#define __VL53L0X_I2C_H

#include "ti_msp_dl_config.h"

#define STATUS_OK   0x00
#define STATUS_FAIL 0x01

void     VL53L0X_i2c_init(void);
uint8_t  VL53L0X_write_multi(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t  VL53L0X_read_multi (uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t  VL53L0X_write_byte (uint8_t addr, uint8_t reg, uint8_t  data);
uint8_t  VL53L0X_write_word (uint8_t addr, uint8_t reg, uint16_t data);
uint8_t  VL53L0X_write_dword(uint8_t addr, uint8_t reg, uint32_t data);
uint8_t  VL53L0X_read_byte (uint8_t addr, uint8_t reg, uint8_t  *data);
uint8_t  VL53L0X_read_word (uint8_t addr, uint8_t reg, uint16_t *data);
uint8_t  VL53L0X_read_dword(uint8_t addr, uint8_t reg, uint32_t *data);

#endif 


