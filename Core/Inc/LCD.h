/**
  ******************************************************************************
  * File Name          : LCD.h
  * Description        : This file provides code for the configuration
  *                      of the LCD instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/


/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
// LCD defines
#define START    0
#define SIGN     1
#define EXFx_f   2
#define EKFx_s   3
#define EKFy_f   4
#define EKFy_s   5
#define EKFphi_f 6
#define EKFphi_s 7
#define GPSacc_f 8
#define GPSacc_s 9
#define GPSx_f   10
#define GPSx_s   11
#define GPSy_f   12
#define GPSy_s   13
#define VEL_f    14
#define VEL_s    15
#define IMU      16

#define START_BIT  254
#define LCD_GAIN   100

#define LED_ALL    10
#define LED_LEFT   11
#define LED_MID    12
#define LED_RIGHT  13
#define LED_NLEFT  14
#define LED_NMID   15
#define LED_NRIGHT 16
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */
void LcdData(uint8_t* tx_data, float* mu, float* accuracy, float* point_current, float V_current,
			float theta_x, float theta_y, uint8_t tracking_first_flag);
/* USER CODE END Prototypes */



/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
