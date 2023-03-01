/*
 * OpenIMU.h
 *
 *  Created on: 2021¦~3¤ë30¤é
 *      Author: ldsc8
 */

#ifndef INC_OPENIMU_H_
#define INC_OPENIMU_H_

#define Gpy1 1.745e-4f
#define	Tpy 1.9393e-4f
#define Gac	2.5e-4f

#define Gdx 0.0f
#define Gdy 0.0f
#define Gdz 0.000524f // need to change to 0.03*PI/180 2021/03/20 by JAY
#define Tdy 0.0f

void SPIdelay(void);
void openIMU(float *theta_x, float *acc, float *gyro, float *theta_y);

#endif /* INC_OPENIMU_H_ */
