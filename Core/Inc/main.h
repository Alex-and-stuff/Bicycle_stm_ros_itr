/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void start_read(uint16_t *RX1, uint16_t *RX2, uint16_t *RX3);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin LL_GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Act_Off_Pin LL_GPIO_PIN_0
#define Act_Off_GPIO_Port GPIOC
#define Act_On_Pin LL_GPIO_PIN_1
#define Act_On_GPIO_Port GPIOC
#define Mx_TX_Pin LL_GPIO_PIN_0
#define Mx_TX_GPIO_Port GPIOA
#define USART_TX_Pin LL_GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin LL_GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Mx_dir_Pin LL_GPIO_PIN_4
#define Mx_dir_GPIO_Port GPIOA
#define LD2_Pin LL_GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define test_pin_Pin LL_GPIO_PIN_5
#define test_pin_GPIO_Port GPIOC
#define EXT_Orange_LED_Pin LL_GPIO_PIN_1
#define EXT_Orange_LED_GPIO_Port GPIOB
#define EXT_Yellow_LED_Pin LL_GPIO_PIN_2
#define EXT_Yellow_LED_GPIO_Port GPIOB
#define EXT_Green_LED_Pin LL_GPIO_PIN_10
#define EXT_Green_LED_GPIO_Port GPIOB
#define RX_1_Pin LL_GPIO_PIN_7
#define RX_1_GPIO_Port GPIOC
#define RX_2_Pin LL_GPIO_PIN_8
#define RX_2_GPIO_Port GPIOC
#define RX_3_Pin LL_GPIO_PIN_9
#define RX_3_GPIO_Port GPIOC
#define SPI_CSG_Pin LL_GPIO_PIN_11
#define SPI_CSG_GPIO_Port GPIOA
#define SPI_CSXM_Pin LL_GPIO_PIN_12
#define SPI_CSXM_GPIO_Port GPIOA
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin LL_GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define define_STM32
#define USE_ROS
#define NOSPEEDMODE
#define RTK_GPS
// remote mode: comment out DEBUGOPENIMU and EKF
// debug  mode:  leave only DEBUOPENIMU and USE_OPENIMU uncommented

#define INIT_MX                             // Initialize MX servo motor
#define REMOTE_START                        // Use remote?
#define CANLOG							    // Use CAN to log data
#define LCD                                 // transmit to lcd
#define VELOCITY                            // Switch mode when time exceeds 1.5m/s
#define USE_OPENIMU						    // Read IMU data
#define CONTROL							    // Start bicycle balancing control
//#define DEBUGOPENIMU					    // In OpenIMU.c for debugging purposes
#define REMOTE_CONTROL				    	// In PDcontrol.c
#define REMOTE_STOP						    // Stop bicycle with remote
//#define EKF							        // Do EKF and tracking
#define USE_TILTLED                         // Bicycle tilt angle indicator LED

#define TESTEKF			1					// no tracking omega and velocity , but record everything.  I think EKF needs to be defined
#define MXSPEEDCTRL		0

#define V_REF			2.0f					//velocity command
#define V_STD			1.49f					//when control
#define SAMPLE_TIME		0.001f
#define EKF_TIME		0.01f

#define CANSAMPLE		30
#define COLSAMPLE		10
#define CANLOGSAMPLE	10
#define REMOTESAMPLE	500
#define BACKSUPTIME		18000				//back support down time (so slow
#define EKFSAMPLE		10					//
#define FIRSAMPLE		10

#define SENSAMPLE		2					//
#define MOTORSAMPLE 	10					//for motor stop need 8~9ms		2020/2/10

#define MXHEX_CENTER	0x7EA				//0x7B8 //0x7A8 Date:2021_0624
#define MXDEC_CENTER	178.242188f			//173.671875f	//181.405		//0x7E0 177.188 2020/09/30
#define MXHEX2DEC		0.08789
#define MXUPPER_BOUND 	MXHEX_CENTER+0x250
#define MXLOWER_BOUND 	MXHEX_CENTER-0x250
#define MXSPD_HEX2RPM	0.114f				// to transfer hex to rpm
#define SPEED_THRESHOLD	0x400
#define REMOTE_UPPER	1968				// 0x7B0
#define REMOTE_CENTER	1513				// 0x5E9
//#define REMOTE_CENTER	1520				// 0x5F0
#define REMOTE_LOWER	1072				// 0x430


#define THETA_COM  		0.8f				// complementary angle unit degree 2020/10/20 1.25, 2021/05/23 0.8
#define REMOTE_ERROR	0.1f				// the remote have something error so give it range

// constant value
#define CAN_TX_GAIN		100
#define CAN_RX_GAIN		0.01f
#define K_GAIN			2.54f	//or 2.45			// by senior
#define epsilon			1.22173f			//unit rad (70 degree)
#define PI 				3.141592f
#define GRAVITY 		9.80665f
#define RAD2DEG			57.2958f
#define DEG2RAD			0.017453f

#define V_MIN           1.6f
#define V_MAX           2.6f
#define W_MIN          -1.4f
#define W_MAX           1.4f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
