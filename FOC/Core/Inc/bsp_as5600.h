// Created by MOS on 2022/12/6.
//

#ifndef STM32_BSP_AS5600_H
#define STM32_BSP_AS5600_H

#endif //STM32_BSP_AS5600_H

#include "i2c.h"

#define AS5600_I2C_HANDLE hi2c2

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1

/*
注意:AS5600的地址0x36是指的是原始7位设备地址,而ST I2C库中的设备地址是指原始设备地址左移一位得到的设备地址
*/

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)


#define AS5600_RESOLUTION 4096 //12bit Resolution

#define AS5600_RAW_ANGLE_REGISTER  0x0C


void bsp_as5600Init(void);
uint16_t bsp_as5600GetRawAngle(void);
float getAngle(void);

float getVelocity(void);

