//
// Created by MOS on 2022/12/6.
//

#include "bsp_as5600.h"

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718
unsigned long velocity_calc_timestamp;
float angle_prev;
static float angle_data_prev = 0; //上次位置
static float full_rotation_offset; //转过的整圈数

void bsp_as5600Init(void) {
    /* init i2c interface */

    /* init var */
    full_rotation_offset = 0;
    angle_data_prev = bsp_as5600GetRawAngle();
}

static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, dev_addr, pData, count, i2c_time_out);
    return status;
}

static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, (dev_addr | 1), pData, count, i2c_time_out);
    return status;
}

uint16_t bsp_as5600GetRawAngle(void) {
    uint16_t raw_angle;
    uint8_t buffer[2] = {0};
    uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;

    i2cWrite(AS5600_ADDR, &raw_angle_register, 1);
    i2cRead(AS5600_ADDR, buffer, 2);
    raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return raw_angle;
}

float getAngle(void) {
    float angle_data = bsp_as5600GetRawAngle();

    float d_angle = angle_data - angle_data_prev;
    if(abs(d_angle) > (0.8 * AS5600_RESOLUTION)) {
        full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
    }
    angle_data_prev = angle_data;

    return (full_rotation_offset + (angle_data / (float)AS5600_RESOLUTION)*_2PI);
}

float getVelocity(void)
{
    unsigned long now_us;
    float Ts, angle_c, vel;

    // calculate sample time
    now_us = SysTick->VAL; //_micros();
    if(now_us<velocity_calc_timestamp)Ts = (float)(velocity_calc_timestamp - now_us)/9*1e-6;
    else
        Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/9*1e-6;
    // quick fix for strange cases (micros overflow)
    if(Ts == 0 || Ts > 0.5) Ts = 1e-3;

    // current angle
    angle_c = getAngle();
    // velocity calculation
    vel = (angle_c - angle_prev)/Ts;

    // save variables for future pass
    angle_prev = angle_c;
    velocity_calc_timestamp = now_us;
    return vel;
}
