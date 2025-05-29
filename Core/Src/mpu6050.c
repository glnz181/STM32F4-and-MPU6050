#include "mpu6050.h"
#include <math.h>
#include "main.h"
#include "i2c.h"
#include "stdio.h"

void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
	uint8_t check;
	uint8_t Data;

	// WHO_AM_I register kontrolü
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

	if (check == 0x68) {
		// Power management register ayarı
		Data = 0;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1,
				1000);

		// Sample rate divider
		Data = 0x07;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1,
				1000);

		// Gyro konfigürasyonu ±500 °/s
		Data = 0x08;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1,
				1000);

		// Accel konfigürasyonu ±4g
		Data = 0x08;
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1,
				1000);
	}
}

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *mpu6050) {
	uint8_t Rec_Data[14];

	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14,
			1000) != HAL_OK) {
		// hata oluştu, debug mesajı bas
		printf("I2C okuma hatası!\r\n");
	}

	mpu6050->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	mpu6050->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	mpu6050->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
	mpu6050->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
	mpu6050->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
	mpu6050->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

	// Dönüşümler
	mpu6050->Ax = mpu6050->Accel_X_RAW / 8192.0;
	mpu6050->Ay = mpu6050->Accel_Y_RAW / 8192.0;
	mpu6050->Az = mpu6050->Accel_Z_RAW / 8192.0;

	mpu6050->Gx = mpu6050->Gyro_X_RAW / 65.5;
	mpu6050->Gy = mpu6050->Gyro_Y_RAW / 65.5;
	mpu6050->Gz = mpu6050->Gyro_Z_RAW / 65.5;

}
