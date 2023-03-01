/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
#define CAN_LOG_CTRL_ID		1
#define CAN_LOG_TRK_ID		2
#define CAN_LOG_EKF_ID		3
#define CAN_LOG_STATE_ID	4
#define CAN_CURRENT_W_ID 	5
#define CAN_LOG_ACC_ID		6
#define CAN_LOG_DATA		  7
#define CAN_LOG_GPS_ID		8
#define CAN_LOG_GYRO_ID		9
#define CAN_V_CMD_ID 		  10
#define CAN_LOG_CROSS_ID	11
#define CAN_LOG_PURE_ID		12
#define CAN_LOG_REF_ID		14
#define CAN_LOG_THETA_ID	15
/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CanLogger(float theta_x, float theta_y, float *gyro, float *acc, float *mu,
		uint8_t check_flag, float remote_angle, float hexgoal,
		float *point_current, float *tracking_control, uint8_t tracking_first_flag,float *stateD, float *accuracy);

void CAN_Filter_Init(void);

static inline void CAN_ClearFlag_ERRI(CAN_HandleTypeDef *hcan)
{
  WRITE_REG(hcan->Instance->MSR, CAN_MSR_ERRI);
}

static inline void TxMSG_Init(CAN_TxHeaderTypeDef* TxMSG) {
	TxMSG->RTR = CAN_RTR_DATA;
	TxMSG->IDE = CAN_ID_STD;
	TxMSG->DLC = 8U;
	TxMSG->TransmitGlobalTime = DISABLE;
}
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

