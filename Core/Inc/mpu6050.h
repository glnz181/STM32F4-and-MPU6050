#ifndef MPU6050_H
#define MPU6050_H
#include "main.h"

#include "stm32f4xx_hal.h"

#define MPU6050_ADDR 0x68 << 1  // yani 0xD0 değil, 0x68 << 1 = 0xD0 oluyor zaten.
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define INT_PIN_CFG_REG 0x37
#define INT_ENABLE_REG 0x38
#define DATA_RDY_EN 0x01

typedef struct {
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	float Ax, Ay, Az;
	float Gx, Gy, Gz;
} MPU6050_t;

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu6050);

#endif
