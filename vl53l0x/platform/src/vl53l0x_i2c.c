/**
 * VL53L0X I2C 硬件层 — MSPM0 DL_I2C
 * TOF_I2C = I2C0, PA0=SDA, PA1=SCL, ~400kHz
 */

#include "vl53l0x_i2c.h"

/* VL53L0X 独占 I2C0 (PA0=SDA, PA1=SCL) */
#ifndef TOF_I2C_INST
#define TOF_I2C_INST I2C0
#endif

/* SysConfig 已配置 TOF_I2C, 这里无需额外 GPIO 初始化 */
void VL53L0X_i2c_init(void)
{
    /* I2C0 已在 SYSCFG_DL_init() 中初始化 */
}

/* 等待 I2C 控制器空闲 (超时 ~50ms) */
static int I2C_WaitIdle(void)
{
    uint32_t timeout = 200000;
    while (!(DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return STATUS_FAIL;
    }
    /* 检查是否有 NACK/仲裁丢失 */
    if (DL_I2C_getControllerStatus(I2C0) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_resetControllerTransfer(I2C0);
        return STATUS_FAIL;
    }
    return STATUS_OK;
}

static void I2C_Delay(void)
{
    for (volatile int i = 0; i < 1000; i++) __NOP();
}

/* ---- 多字节写: START → Addr(W) → [Reg + Data...] → STOP ---- */
uint8_t VL53L0X_write_multi(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx_buf[65]; /* 1 reg + up to 64 data */
    tx_buf[0] = reg;
    for (uint16_t i = 0; i < len; i++) tx_buf[i + 1] = buf[i];

    DL_I2C_fillControllerTXFIFO(I2C0, tx_buf, len + 1);
    DL_I2C_startControllerTransfer(I2C0, addr >> 1,
        DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);
    int r = I2C_WaitIdle();
    I2C_Delay();
    return r;
}

/* ---- 多字节读: START → Addr(W) → Reg → STOP, 再 START → Addr(R) → Data... → STOP ---- */
uint8_t VL53L0X_read_multi(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* Phase 1: 写寄存器地址 (独立事务, 带 STOP) */
    uint8_t reg_buf[1] = {reg};
    DL_I2C_fillControllerTXFIFO(I2C0, reg_buf, 1);
    DL_I2C_startControllerTransfer(I2C0, addr >> 1,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    if (I2C_WaitIdle() != STATUS_OK) return STATUS_FAIL;

    /* Phase 2: 读数据 (独立事务, 带 STOP) */
    DL_I2C_startControllerTransfer(I2C0, addr >> 1,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    if (I2C_WaitIdle() != STATUS_OK) return STATUS_FAIL;

    for (uint16_t i = 0; i < len; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C0));
        buf[i] = DL_I2C_receiveControllerData(I2C0);
    }
    I2C_Delay();
    return STATUS_OK;
}

/* ---- 单字节写 ---- */
uint8_t VL53L0X_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    return VL53L0X_write_multi(addr, reg, &data, 1);
}

/* ---- 双字节写 (大端). 奇数地址须拆成两次单字节写 ---- */
uint8_t VL53L0X_write_word(uint8_t addr, uint8_t reg, uint16_t data)
{
    uint8_t buf[2];
    if (reg & 0x01) {
        buf[0] = (uint8_t)(data >> 8);
        if (VL53L0X_write_multi(addr, reg, buf, 1) != STATUS_OK) return STATUS_FAIL;
        buf[0] = (uint8_t)(data & 0xFF);
        return VL53L0X_write_multi(addr, reg + 1, buf, 1);
    }
    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFF);
    return VL53L0X_write_multi(addr, reg, buf, 2);
}

/* ---- 四字节写 (大端) ---- */
uint8_t VL53L0X_write_dword(uint8_t addr, uint8_t reg, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)((data >> 16) & 0xFF);
    buf[2] = (uint8_t)((data >> 8) & 0xFF);
    buf[3] = (uint8_t)(data & 0xFF);
    return VL53L0X_write_multi(addr, reg, buf, 4);
}

/* ---- 单字节读 ---- */
uint8_t VL53L0X_read_byte(uint8_t addr, uint8_t reg, uint8_t *data)
{
    return VL53L0X_read_multi(addr, reg, data, 1);
}

/* ---- 双字节读 (大端) ---- */
uint8_t VL53L0X_read_word(uint8_t addr, uint8_t reg, uint16_t *data)
{
    uint8_t buf[2];
    if (VL53L0X_read_multi(addr, reg, buf, 2) != STATUS_OK) return STATUS_FAIL;
    *data = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return STATUS_OK;
}

/* ---- 四字节读 (大端) ---- */
uint8_t VL53L0X_read_dword(uint8_t addr, uint8_t reg, uint32_t *data)
{
    uint8_t buf[4];
    if (VL53L0X_read_multi(addr, reg, buf, 4) != STATUS_OK) return STATUS_FAIL;
    *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] << 8)  | (uint32_t)buf[3];
    return STATUS_OK;
}
