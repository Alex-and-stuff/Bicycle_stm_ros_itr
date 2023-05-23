/*
 * OpenIMU.c
 *
 *  Created on: 2021�~3��30��
 *      Author: ldsc8
 */

#include "spi.h"
#include "OpenIMU.h"

void SPIdelay() { 				// SPI write need to some time
	int i = 0;
	while (i <= 500) {
		LL_GPIO_TogglePin(GPIOA, LD2_Pin);
		i = i + 1;
	}
}

void openIMU(float *theta_x, float *acc, float *gyro, float *theta_y) {
	uint8_t NUL[2] = { 0x00, 0x00 };
	uint8_t Reno[2]; //Receive nothing
	uint8_t theta_address[2] = { 0x00, 0x3D };						//AHRS data
	uint8_t Stat_theta[2];
	uint8_t tmp[2];							//SPI receive data higher than 8 bit
	uint8_t Temperature_theta[2];
	LL_GPIO_ResetOutputPin(GPIOA, SPI_CSG_Pin);				//start spi talking

	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &theta_address, (uint8_t*) &Reno,1U, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &NUL, (uint8_t*) &Stat_theta, 1U,HAL_MAX_DELAY);		//receive data
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &NUL, (uint8_t*) &tmp, 1U,HAL_MAX_DELAY);
	int16_t Rxtx = tmp[1] << 8 | tmp[0]; //roll
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &NUL, (uint8_t*) &tmp, 1U,HAL_MAX_DELAY);
	int16_t Rxty = tmp[1] << 8 | tmp[0]; //pitch
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &NUL, (uint8_t*) &tmp, 1U,HAL_MAX_DELAY);
//	int16_t Rxtz = tmp[1] << 8 | tmp[0]; //yaw
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &NUL,(uint8_t*) &Temperature_theta, 1U, HAL_MAX_DELAY);
//	SPIdelay();
	LL_GPIO_SetOutputPin(GPIOA, SPI_CSG_Pin);				//end spi talking
	*theta_x = Rxtx * Tpy + Tdy;
	*theta_y = -(Rxty * Tpy + Tdy);
//	*theta_z = Rxtz * Tpy + Tdy;

	SPIdelay();
	uint8_t imu_address[2] = {0x00,0x3E};										//AHRS data
	uint8_t Stat[2];
	uint8_t Rxgx[2];
	uint8_t Rxgy[2];
	uint8_t Rxgz[2];
	uint8_t Rxax[2];
	uint8_t Rxay[2];
	uint8_t Rxaz[2];
	uint8_t Temperature[2];

	LL_GPIO_ResetOutputPin(GPIOA, SPI_CSG_Pin);				//start spi talking
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&imu_address, (uint8_t*)&Reno, 1U, HAL_MAX_DELAY);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Stat, 1U, HAL_MAX_DELAY);	//receive data
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Rxgx, 1U, HAL_MAX_DELAY);
	int16_t Rxgx_dot = Rxgx[1] << 8 | Rxgx[0];
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Rxgy, 1U, HAL_MAX_DELAY);
	int16_t Rxgy_dot = Rxgy[1] << 8 | Rxgy[0];
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Rxgz, 1U, HAL_MAX_DELAY);
	int16_t Rxgz_dot = Rxgz[1] << 8 | Rxgz[0];
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Rxax, 1U, HAL_MAX_DELAY);
	int16_t Rxax_dot = Rxax[1] << 8 | Rxax[0];
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Rxay, 1U, HAL_MAX_DELAY);
	int16_t Rxay_dot = Rxay[1] << 8 | Rxay[0];
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Rxaz, 1U, HAL_MAX_DELAY);
	int16_t Rxaz_dot = Rxaz[1] << 8 | Rxaz[0];
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&NUL, (uint8_t*) &Temperature, 1U,HAL_MAX_DELAY);
	LL_GPIO_SetOutputPin(GPIOA, SPI_CSG_Pin);				//end spi talking

	gyro[0] = Rxgx_dot*Gpy1+Gdx;
	gyro[1] = -(Rxgy_dot*Gpy1+Gdy);
	gyro[2] = -(Rxgz_dot*Gpy1+Gdz);

	acc[0] = Rxax_dot*Gac*GRAVITY;
	acc[1] = Rxay_dot*Gac*GRAVITY;
	acc[2] = Rxaz_dot*Gac*GRAVITY;




#ifdef DEBUGOPENIMU
	// acc:   [ax, ay, az]
	// gyro:  [roll_d, pitch_d, yaw_d]

//	int aa = *theta_x * (180/PI)*1000;
//	int aa = acc[0]*1000;
//	int bb = gyro[0]*(180/PI)*1000;
//	int bb = *theta_y * (180/PI)*1000;
//	printf("%d,%d\r\n", aa, bb);

	int cc = acc[2]*1000;
//	int bb = gyro[1]*1000 * (180/PI);
//	int cc = (gyro[2]/cosf(*theta_x))*1000 * (180/PI);
//	printf("%d,%d,%d\r\n",aa,bb,cc);
	printf("%d\r\n",cc);
#endif
}
