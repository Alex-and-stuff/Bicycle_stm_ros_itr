/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"
#include <stdio.h>
#include "OpenIMU.h"
#include "sensor_fusion.h"
#include "MxMotor.h"
#include "dynamixel.h"
#include "controller.h"
#include "EKF.h"
#include "pure_pursuit.h"
#include "Tracking.h"
#include "LCD.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int i = 0;

CAN_TxHeaderTypeDef TxMSG;
CAN_RxHeaderTypeDef RxMSG;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
float V_current;
uint8_t gpsdata[36];  // for details, read stm32f4xx_it.c usart3
uint8_t update_gps;
float velocity_cmd = V_REF;
float imu_bias = 0;
uint8_t emergency_brake_flag = 0;
uint8_t velocity_cmd_flag = 0;		// Remote on mean velocity > 0
uint8_t tracking_first_flag = 0;		// to recoder the first shift time	//Remember

float delta_ref = MXHEX_CENTER* PI / 180.0f;
float theta_x = 0.0f;
float theta_y = 0.0f;
float acc[3]  = { 0.0f, 0.0f, -GRAVITY };
float gyro[3] = { 0.0f };

float state[3] = {0};
float tracking_control[2] = { 0 };                  // tracking output (1):velocity (m/s) (2):omega (rad/s)
// ============= Remote Parameters ===============
uint16_t RX1 = REMOTE_CENTER;
uint16_t RX2 = REMOTE_CENTER;
uint16_t RX3 = REMOTE_LOWER;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t check_sum(void);	// check the DMA GPS data checkSum
void EmergencyBrake(void);	// let the vehicle stop
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_Init1msTick(180000000);			//Add here to fix MX generated code
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SYSTICK_EnableIT();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_TIM14_Init();
  MX_TIM12_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
#ifdef USE_ROS
  setup();
  HAL_TIM_Base_Start_IT(&htim14);
#endif
//  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim12);

  // ========== INITIALIZE CAN =======================
  CAN_Filter_Init();
  TxMSG_Init(&TxMSG);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
   while (1);
  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // =========== INTERRUPT ENABLE ====================
  LL_USART_EnableIT_RXNE(UART4);
  LL_TIM_EnableIT_CC1(TIM2);
  LL_USART_EnableIT_RXNE(USART3);
  LL_TIM_CC_EnableChannel(TIM8,	TIM_CCER_CC1E | TIM_CCER_CC2E |
     TIM_CCER_CC3E | TIM_CCER_CC4E);	// Enable all PWM input port
  LL_TIM_EnableIT_CC1(TIM8);			// Enable PWM input capture interrupt
  LL_TIM_EnableCounter(TIM8);			// Start timer8 for PWM input capture
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_8);
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_8);

  // ============ GPS DMA SETUP =======================
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1, LL_USART_DMA_GetRegAddr(USART3));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t) gpsdata);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, sizeof(gpsdata));

  // ============ HAL Time Interrupt Initialize =======
  uint32_t now = 0;
  uint32_t oldlog_time = 0;
  uint32_t oldfir_time = 0;
  uint32_t oldcan_time = 0;
  uint32_t olddebug_time = 0;
  uint32_t oldcol_time = 0;
  uint32_t oldekf_time = 0;
  uint32_t oldrmt_time = 0;
  uint32_t time_start = 0;				// to record back support down time
  uint32_t time_stop = 0;

  // ============ Flag Parameter Initialize ===========
  uint8_t vel_first_flag = 0;			// velocity > limit minimum velocity (1.5)
//  uint8_t velocity_cmd_flag = 0;		// Remote on mean velocity > 0
  uint8_t check_flag = 0; 				// GPS check sum
  uint8_t turn_circle_flag = 0;			// tracking line or arc
//  uint8_t tracking_first_flag = 0;		// to recoder the first shift time	//Remember
//  uint8_t emergency_brake_flag = 0;		// Emergency Brake System
//
  // ============ Tracking Parameter Initialize =======
  float way_point_begin[2] = { 60.2217f, 8.9381f};	  // tracking begin way_point
  float way_point_end[2]   = { 10.06f, 32.6295f };    // tracking end way_point
  float circle_point[2]    = { 13.2039f, 39.2861f };  // if tracking trajectory is arc
  float radius = 7.3617f;					          // tracking trajectory arc radius
//  float tracking_control[2] = { 0 };                  // tracking output (1):velocity (m/s) (2):omega (rad/s)
  float stateD[3] = { 0.0f };
//  float imu_bias = 0;
  uint8_t line = 3;									  // tracking stage
  uint8_t stage = 8;
  uint32_t time_shift = 0;	                          // normalize time in tracking process each stage

//  // ============= Remote Parameters ===============
//  uint16_t RX1 = REMOTE_CENTER;
//  uint16_t RX2 = REMOTE_CENTER;
//  uint16_t RX3 = REMOTE_LOWER;

  // ============= CAN Parameters ==================
//  float velocity_cmd = V_REF;

//  // ============= IMU Parameters ==================
//  float theta_x = 0.0f;
//  float theta_y = 0.0f;
//  float acc[3]  = { 0.0f, 0.0f, -GRAVITY };
//  float gyro[3] = { 0.0f };
//
  // ============= Controller Parameters ===========
  float remote_control = 0.0f;
  float delta_goal     = 0.0f;

  // ============= EKF Parameters ==================
  float init_x = 79.5f;								//the vehicle initial point
  float init_y = -3.0f;
  float init_phi = 2.7f;
  float point_current[2] = { init_x, init_y };
  float gps_drift[2] = {0.0f};
  float horizontal_accuracy = 0.0f;
  float vertical_accuracy = 0.0f;
  double lat_current;
  double lon_current;
  float mu[4] = { init_x, init_y, init_phi, 0.0f }; // X Y Phi velocity

  // ============= LCD Parameters ==================
  uint8_t LCD_tx[17] = {0};


  // ============= MX Motor Initialization ========
#ifdef INIT_MX
  HAL_Delay(1000);							//wait init 1(second)
#if MXSPEEDCTRL == 0
  AngleLimit(MXUPPER_BOUND, MXLOWER_BOUND); // setup the Mx motor limit angle
//	PIDsetup(185,115,15);					// Mx motor setup PID gain
  Motor();									// so min control angle 0.088 degree
#endif
#if MXSPEEDCTRL == 1
  AngleLimit(0x00, 0x00);
  HAL_Delay(1000);
  dxl_write_word(MOTOR_ID, P_GOAL_SPEED_L, 0x00); // >0x400 is turn left
  printf("Hello this is Mx motor speed mode\r\n");
#endif
//	Torque_off();
#endif

  // ====== Lower Back Support Of Bicycle ========
#ifdef REMOTE_START
  LL_GPIO_ResetOutputPin(GPIOC, Act_Off_Pin);	// down
  LL_GPIO_SetOutputPin(GPIOC, Act_On_Pin);
#endif

  // ============= LCD Initialization ========
  HAL_HalfDuplex_EnableTransmitter(&huart5); 	// LCD display
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	// ================== Start CAN Logger And LCD ================
#ifdef CANLOG
	now = HAL_GetTick();
	uint16_t logdt = now - oldlog_time;
	if (logdt >= CANLOGSAMPLE) {
		oldlog_time = HAL_GetTick();
		float accuracy[2] = {vertical_accuracy, horizontal_accuracy};
		CanLogger(theta_x, theta_y, gyro, acc, mu, check_flag,
				remote_control, delta_goal, point_current, tracking_control,
				tracking_first_flag, stateD, accuracy); // need final test

		// ============ Pass LCD Data At Every Can Tick ============
#ifdef LCD
		LcdData(LCD_tx, mu, accuracy, point_current, V_current, theta_x, theta_y, tracking_first_flag);
		HAL_UART_Transmit(&huart5, LCD_tx, sizeof(LCD_tx), 2000);
#endif
	}
#endif

	//  ========================== Remote control ==========================
	if(!velocity_cmd_flag){
	    estimator(theta_x, gyro[0], &imu_bias);
	}

	if (!vel_first_flag) {
#ifdef VELOCITY
#ifdef REMOTE_START
		if (RX3 >= REMOTE_UPPER * (1 - REMOTE_ERROR) && RX3 <= REMOTE_UPPER * (1 + REMOTE_ERROR)) {//if remote is on
			// Activates when the switch on the side is switched up! Turn the velocity_cmd_flag on
			velocity_cmd = V_REF;
			velocity_cmd_flag = 1;
		} else {
			velocity_cmd = 0.0f;
			velocity_cmd_flag = 0;
		}
#endif
		// =========== Lift Back Support Of Bicycle At V_STD ==============
		if (V_current > V_STD) {
			vel_first_flag = 1;							//@TODO maybe need to check v0 > 1.5(m/s) if velocity have peak
			LL_GPIO_SetOutputPin(GPIOC, Act_Off_Pin);	//up
			LL_GPIO_SetOutputPin(GPIOC, Act_On_Pin);
		}

#ifdef NOSPEEDMODE
		if(velocity_cmd_flag)vel_first_flag = 1;
#endif

#endif
	}else{
#ifdef REMOTE_STOP
		// ============== Situation Where The Remote Is Switched Off =========
		if (RX3 <= REMOTE_LOWER * (1 + REMOTE_ERROR) && RX3 >= REMOTE_LOWER * (1 - REMOTE_ERROR)) {
			// ========== Lower back support ==================================
			LL_GPIO_ResetOutputPin(GPIOC, Act_Off_Pin);	//down
			LL_GPIO_SetOutputPin(GPIOC, Act_On_Pin);

			// ========== Count down time till back support is fully lowered, then stop the vehicle
			if (time_start == 0) {
				time_start = HAL_GetTick();
			} else {
				time_stop = HAL_GetTick();
				uint16_t cntdt = time_stop - time_start;
				if (cntdt > BACKSUPTIME) {		// to calculate 18 second
					emergency_brake_flag = 1;
				}
			}
		}
#endif
	}

#if TESTEKF == 1
	if (RX2 <= REMOTE_LOWER * (1 + REMOTE_ERROR) && RX2 >= (1 - REMOTE_ERROR) * REMOTE_LOWER) {
		// Pull down the trigger to lift the back support up
		LL_GPIO_SetOutputPin(GPIOC, Act_Off_Pin);
		LL_GPIO_SetOutputPin(GPIOC, Act_On_Pin);
	}else{
		if(velocity_cmd_flag) {
			velocity_cmd = V_REF;
			tracking_control[0] = V_REF;
		}
	}
#endif

	// Stop bicycle by pulling up the controller trigger (Actual breaking system!!)
	if (RX2 <= REMOTE_UPPER * (1 + REMOTE_ERROR) && RX2 >= REMOTE_UPPER * (1 - REMOTE_ERROR)){
		emergency_brake_flag = 1;
	}
	if(emergency_brake_flag) EmergencyBrake();


	// ================= Read IMU Data Through SPI2 =======================
#ifdef USE_OPENIMU
	if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_8)) {  // Read IMU data when external interrupt triggered from IMU
		openIMU(&theta_x, acc, gyro, &theta_y);
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
	}
#endif

	// ================= IMU Position Display (LED Shield) =================
#ifdef USE_TILTLED
	if (tracking_first_flag == 0){
		// light signal (shield LED)
		if (theta_x == 0 && theta_y == 0){                       	   // if no IMU data is received, make all three lights light up
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 ,GPIO_PIN_SET); 	   // Green LED (right)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 ,GPIO_PIN_SET);  	   // Red   LED (middle)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET);  	   // Green LED (left)
		}
		else{
			double angle_margin = 0.02;
			if (theta_x > (0.0 + angle_margin)){                       // if the bicycle is about horizontal, light up the red LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 ,GPIO_PIN_RESET);  // Green LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 ,GPIO_PIN_SET);    // Red   LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET);  // Green LED
			}
			else if (theta_x < (0.0 - angle_margin)){                  // if the bicycle is tilted left (about x-axis), light up the left green LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 ,GPIO_PIN_RESET);  // Green LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 ,GPIO_PIN_RESET);  // Red   LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET);    // Green LED
			}
			else{                                                      // if the bicycle is tilted right (about x-axis), light up the right green LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 ,GPIO_PIN_SET);    // Green LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 ,GPIO_PIN_RESET);  // Red   LED
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET);  // Green LED
			}
		}
	}
#endif

	// ========================== Open RTK GPS ==========================
#ifdef RTK_GPS
	now = HAL_GetTick();
	uint16_t ekfdt = now - oldekf_time;
	if (ekfdt >= EKFSAMPLE) {
		if(update_gps && velocity_cmd_flag){        // First check if the GPS data is correct
			check_flag = check_sum();
			if (check_flag != 3) {
				update_gps = 0;						// if check sum error (not equal to 3), don't update GPS
			}
		}
		if (update_gps) {							// Process the "correct" GPS data
			readGPS(&lat_current, &lon_current);
			gpsXY(lat_current, lon_current, point_current);
			ReadAccuracyEstimate(&horizontal_accuracy,&vertical_accuracy);
		}
		if (velocity_cmd_flag) {					// "velocity_cmd_flag":= The bicycle is moving
			if(update_gps){
				point_current[0] = point_current[0] + gps_drift[0];
				point_current[1] = point_current[1] + gps_drift[1];
			}
			// ========== Perform The EKF Algorithm For Localization ===========
//			EKF_filter(mu, theta_x, acc, gyro, point_current, V_current, theta_y, horizontal_accuracy);
		} else {
			gps_drift[0] = init_x - point_current[0];  // Seems like the distance difference instead of drift?
			gps_drift[1] = init_y - point_current[1];
		}
	}
#endif

	// ================ EKF Mode, Pass current velocity to EKF ===============
#ifndef EKF
	mu[3] = V_current;  // Seems unnecessary?
#endif

// ======================= Read MX Motor Position ======================
	int delta_refu16_tmp = 0;
	uint8_t position_state;
	delta_refu16_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
	position_state = dxl_get_result();
	if (position_state == COMM_RXSUCCESS) {
		if (delta_refu16_tmp < MXUPPER_BOUND && delta_refu16_tmp > MXLOWER_BOUND) {
			delta_ref = (float) (MXHEX_CENTER - delta_refu16_tmp) * MXHEX2DEC * DEG2RAD;
		}
	}

	// ====================== Compute state ======================
	state[0] = theta_x;
	state[1] = gyro[0];
	state[2] = delta_ref;

	// ================ Bicycle Control ======================================
#ifdef CONTROL
	now = HAL_GetTick();
	uint16_t coldt = now - oldcol_time;
	if (coldt >= COLSAMPLE && vel_first_flag) {
		oldcol_time = HAL_GetTick();
#if MXSPEEDCTRL == 0
		float disx = powf(mu[0] - stateD[0], 2);  // Distance between "current state" and "tracking virtual state"
		float disy = powf(mu[1] - stateD[1], 2);
		float dis_mu2state = sqrtf(disx + disy);
		uint8_t dis_flag = dis_mu2state > 0.5f ? 1 : 0;
		dis_flag = line == 3 ? dis_flag : 0;
//		PDControl(theta_x, gyro[0], tracking_control[1], &remote_control, &delta_goal,dis_flag, imu_bias);
//		LMII(theta_x, gyro[0], tracking_control[0], tracking_control[1], &remote_control, &delta_goal, V_current,dis_flag, imu_bias);

//		LMIII(state, V_current, tracking_control, RX1);
		PDD(state, V_current, tracking_control, RX1, dis_flag);
#endif

#if MXSPEEDCTRL == 1
		float disx = powf(mu[0] - stateD[0], 2);
		float disy = powf(mu[1] - stateD[1], 2);
		float dis_mu2state = sqrtf(disx + disy);
		uint8_t dis_flag = dis_mu2state > 0.5f ? 1 : 0;
		dis_flag = line == 3 ? dis_flag : 0;
		PDControl(theta_x, gyro[0], tracking_control[1], &remote_control, &delta_goal,dis_flag);
		controldesign(theta_x, gyro[0], tracking_control[1], &remote_control, &delta_goal, mu[3]);
#endif
	}
#endif

	float acc_x = acc[0] + GRAVITY*sinf(theta_y);
	float position_omega = gyro[2]/cosf(theta_x);
	loop2(&theta_x, &gyro[0], &delta_ref, &V_current, tracking_control, acc_x, position_omega, &lat_current, &lon_current, &horizontal_accuracy, (int)update_gps, point_current);

//	printf("vel: %d\r\n", (int)(V_current*10));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_PWR_EnableOverDriveMode();
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 360, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(180000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		LL_USART_TransmitData8(USART2, (uint8_t) *ptr++);
		while (LL_USART_IsActiveFlag_TXE(USART2) == RESET)
			;
	}
	return len;
}

void start_read(uint16_t *RX1, uint16_t *RX2, uint16_t *RX3) {
	*RX1 = LL_TIM_IC_GetCaptureCH1(TIM8);
	*RX2 = LL_TIM_IC_GetCaptureCH3(TIM8);
	*RX3 = LL_TIM_IC_GetCaptureCH4(TIM8);	// *this is correct to read CH3
}

uint8_t check_sum() {	//checkSum A OK is one , checkSum B OK is two , if all OK return three
	uint8_t flag = 0;
	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	for (int i = 2; i < sizeof(gpsdata) - 2; i++) {
		CK_A = CK_A + gpsdata[i];
		CK_B = CK_B + CK_A;
	}
	if (CK_A == gpsdata[sizeof(gpsdata) - 2]) {
		flag = 2;
	}
	if (CK_B == gpsdata[sizeof(gpsdata) - 1]) {
		flag++;
	}
	return flag;
}

void EmergencyBrake(void){	// make vehicle stop and shut down system

	float velocity_cmd_stop =0.1f; // the FOC v_cmd 0-m/s is 0 Torque

	uint32_t now = 0;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		uint16_t v_cmd;
		v_cmd = velocity_cmd_stop * CAN_TX_GAIN;
		TxData[1] = v_cmd >> 8U;
		TxData[0] = v_cmd;
		TxMSG.StdId = CAN_V_CMD_ID;
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
//		printf("yoyoyo\r\n");
	}
	HAL_Delay(15000);	// let the vehicle stop and receive nothing command
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// ============= Publish/Subscribe from ROS ==================
    if (htim->Instance == htim14.Instance)
    {
//    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
//    	i ++;
//    	int a = i%120;
//#ifdef USE_ROS
//    	int b = (int)delta_ref*100;
//    	loop(&a);
//    	loop2(&theta_x, &gyro[0], &delta_ref, &V_current, tracking_control);
//#endif
    }

   	// ======= Transmit v_tmp Through CAN Bus To Rear Wheel ========
	if (htim->Instance == htim12.Instance)
	{
		if(!emergency_brake_flag){
#ifdef VELOCITY
			// ======= Transmit v_tmp Through CAN Bus To Rear Wheel ========
			float v_tmp = velocity_cmd;
			if(!velocity_cmd_flag){
				v_tmp = 0;
			}
			// ======= Disable Velocity Command In TESTEKF Mode ========
#if TESTEKF == 0
			if(tracking_first_flag){
				v_tmp = tracking_control[0];
				if (tracking_control[0] > V_MAX) v_tmp = V_MAX;
				if (tracking_control[0] < V_MIN) v_tmp = V_MIN;
			}
#endif

#ifdef NOSPEEDMODE
			v_tmp = 0;
#endif
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
				uint16_t v_cmd;
				v_cmd = v_tmp * CAN_TX_GAIN;
				TxData[1] = v_cmd >> 8U;
				TxData[0] = v_cmd;
				TxMSG.StdId = CAN_V_CMD_ID;
				HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
			}
#endif
		}
	}

    // ================== Remote Control Interrupt 10Hz ======================
    if (htim->Instance == htim11.Instance)
	{
    	start_read(&RX1, &RX2, &RX3);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
