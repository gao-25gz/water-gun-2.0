#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

typedef struct
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float rate_x;
    float rate_y;
    float rate_z;
    float pitch;
    float roll;
    float yaw;
    /* Raw filtered angles before the user zero reference. */
    float raw_pitch;
    float raw_roll;
    float raw_yaw;
    uint32_t timestamp_ms;
    uint32_t frame_count;
    uint8_t valid;
} MPU6050_AttitudeData;

void MX_I2C1_Init(void);
void MPU6050_Init(void);
void MPU6050_Poll(void);
uint8_t MPU6050_GetAttitude(MPU6050_AttitudeData *data);
void MPU6050_CalibrateZero(void);

#endif
