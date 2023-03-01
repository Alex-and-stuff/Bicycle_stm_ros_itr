/*
 * CanLogger.c
 *
 *  Created on: 2021�~3��28��
 *      Author: ldsc8
 */

#include "can.h"
#include "MxMotor.h"
#include "main.h"
#include "math.h"
#include "Tracking.h"

extern CAN_TxHeaderTypeDef TxMSG;
extern CAN_RxHeaderTypeDef RxMSG;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern uint32_t TxMailbox;
extern float V_current;

void CanLogger(float theta_x, float theta_y, float *gyro, float *acc, float *mu,
		uint8_t check_flag, float remote_angle, float hexgoal,
		float *point_current, float *tracking_control, uint8_t tracking_first_flag, float *stateD, float *accuracy) {

	int can_gain = CAN_TX_GAIN;
	int16_t Mxspeed = 0;		// Mx106 motor speed
	int16_t Mxspeed_tmp = 0;
	uint8_t rs485_state = 0;
	uint32_t now = 0;			// for clock the time to break infinite loop
//	printf("%.4f\r\n", V_current);
//	printf("%d\r\n", (int)(V_current*100.0));

#ifdef INIT_MX
	int delta_refu16_tmp = 0;		// delta angle type uint16_t
	float delta_ref = MXHEX_CENTER;
	uint8_t position_state;
	delta_refu16_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
	position_state = dxl_get_result();
	if (position_state == COMM_RXSUCCESS) {
		if (delta_refu16_tmp < MXUPPER_BOUND && delta_refu16_tmp > MXLOWER_BOUND) {
			delta_ref = (float) (MXHEX_CENTER - delta_refu16_tmp) * MXHEX2DEC * DEG2RAD;			// get the delta angle unit(rad)
		}
	}

	Mxspeed_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_SPEED_L);
	rs485_state = dxl_get_result();
	if (rs485_state == COMM_RXSUCCESS) {
		if (Mxspeed_tmp >= SPEED_THRESHOLD) {
			Mxspeed = -(Mxspeed_tmp - SPEED_THRESHOLD);
		} else {
			Mxspeed = Mxspeed_tmp;
		}
	}

	int16_t can_theta     = (theta_x + THETA_COM * DEG2RAD) * can_gain * 10;	//right positive left negative
	int16_t can_theta_dot = gyro[0]   * can_gain *10;
	int16_t can_delta     = delta_ref * can_gain *10;	//right negative left positive
	int16_t can_delta_dot = -Mxspeed  * can_gain * MXSPD_HEX2RPM * 2 * PI / 60 *10;// unit(rad/s)
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[7] = can_delta_dot >> 8U;
		TxData[6] = can_delta_dot;
		TxData[5] = can_delta >> 8U;
		TxData[4] = can_delta;
		TxData[3] = can_theta_dot >> 8U;
		TxData[2] = can_theta_dot;
		TxData[1] = can_theta >> 8U;
		TxData[0] = can_theta;
		TxMSG.StdId = CAN_LOG_THETA_ID;  // number 15
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}
#endif

	int16_t can_px  = mu[0] * can_gain;
	int16_t can_py  = mu[1] * can_gain;
	int16_t can_phi = mu[2] * can_gain *10;
	int16_t can_vel = mu[3] * can_gain;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[7] = can_vel >> 8U;
		TxData[6] = can_vel;
		TxData[5] = can_phi >> 8U;
		TxData[4] = can_phi;
		TxData[3] = can_py >> 8U;
		TxData[2] = can_py;
		TxData[1] = can_px >> 8U;
		TxData[0] = can_px;
		TxMSG.StdId = CAN_LOG_EKF_ID;  // number 3
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}


	int16_t can_accx   = (acc[0]+0.0005) * can_gain * 10;	// time 10 for resolution
	int16_t can_accy   = acc[1]  * can_gain;
	int16_t can_accz   = acc[2]  * can_gain;
	int16_t can_thetax = theta_x * can_gain;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[7] = can_thetax >> 8U;
		TxData[6] = can_thetax;
		TxData[5] = can_accz >> 8U;
		TxData[4] = can_accz;
		TxData[3] = can_accy >> 8U;
		TxData[2] = can_accy;
		TxData[1] = can_accx >> 8U;
		TxData[0] = can_accx;
		TxMSG.StdId = CAN_LOG_ACC_ID;  // number 6
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}


	int16_t can_gyrox = gyro[0] * can_gain *10;
	int16_t can_gyroy = gyro[1] * can_gain *10;
	int16_t can_gyroz = gyro[2] * can_gain *10;
	int16_t can_thetaz = (theta_y+0.0005) * can_gain * 10;	// time 10 for resolution (=*1000)
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[7] = can_thetaz >> 8U;
		TxData[6] = can_thetaz;
		TxData[5] = can_gyroz >> 8U;
		TxData[4] = can_gyroz;
		TxData[3] = can_gyroy >> 8U;
		TxData[2] = can_gyroy;
		TxData[1] = can_gyrox >> 8U;
		TxData[0] = can_gyrox;
		TxMSG.StdId = CAN_LOG_GYRO_ID;  // number 9
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}



	int16_t can_reangle   = remote_angle * can_gain * 10; // time 10 for resolution
	int16_t can_hexgoal   = hexgoal      * can_gain *10;
	int16_t can_check_sum = check_flag   * can_gain;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[5] = can_check_sum >> 8U;
		TxData[4] = can_check_sum;
		TxData[3] = can_hexgoal >> 8U;
		TxData[2] = can_hexgoal;
		TxData[1] = can_reangle >> 8U;
		TxData[0] = can_reangle;
		TxMSG.StdId = CAN_LOG_CTRL_ID;  // number 1
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}


	int16_t can_gpsx = point_current[0] * can_gain;
	int16_t can_gpsy = point_current[1] * can_gain;
	int16_t can_Vcur = V_current        * can_gain;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[5] = can_Vcur >> 8U;
		TxData[4] = can_Vcur;
		TxData[3] = can_gpsy >> 8U;
		TxData[2] = can_gpsy;
		TxData[1] = can_gpsx >> 8U;
		TxData[0] = can_gpsx;
		TxMSG.StdId = CAN_LOG_REF_ID;  // number 14
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}


	int16_t can_trackingVd   = tracking_control[0] * can_gain;
	int16_t can_trackingWd   = tracking_control[1] * can_gain *10;
	int16_t can_trackingFlag = tracking_first_flag * can_gain;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[5] = can_trackingFlag >> 8U;
		TxData[4] = can_trackingFlag;
		TxData[3] = can_trackingWd >> 8U;
		TxData[2] = can_trackingWd;
		TxData[1] = can_trackingVd >> 8U;
		TxData[0] = can_trackingVd;
		TxMSG.StdId = CAN_LOG_TRK_ID;  // number 2
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}

	int16_t data_vel   = V_current * can_gain;
	int16_t data_gyroz = gyro[2]   * can_gain *10;
	int16_t data_accx  = acc[0]    * can_gain;
	static float intergal_velocity = 0.0f;
	intergal_velocity = intergal_velocity + (acc[0] + GRAVITY*sinf(theta_y))*CANLOGSAMPLE*SAMPLE_TIME;
	int16_t data_inter_vel = intergal_velocity * can_gain;

	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[7] = data_inter_vel >> 8U;
		TxData[6] = data_inter_vel;
		TxData[5] = data_accx >> 8U;
		TxData[4] = data_accx;
		TxData[3] = data_gyroz >> 8U;
		TxData[2] = data_gyroz;
		TxData[1] = data_vel >> 8U;
		TxData[0] = data_vel;
		TxMSG.StdId = CAN_LOG_DATA;  // number 7
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}

	int16_t can_X = stateD[0] * can_gain;
	int16_t can_Y = stateD[1] * can_gain;
	int16_t can_Z = stateD[2] * can_gain * 10;
	static float intergal_omega = 0.0f;
	intergal_omega = intergal_omega + (gyro[2]/cosf(theta_x+THETA_COM*(PI/180.0f)))*CANLOGSAMPLE*SAMPLE_TIME;
	int16_t data_inter_theta = intergal_omega * can_gain *10;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[7] = data_inter_theta >> 8U;
		TxData[6] = data_inter_theta;
		TxData[5] = can_Z >> 8U;
		TxData[4] = can_Z;
		TxData[3] = can_Y >> 8U;
		TxData[2] = can_Y;
		TxData[1] = can_X >> 8U;
		TxData[0] = can_X;
		TxMSG.StdId = CAN_LOG_STATE_ID;  // number 4
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}

	int16_t can_ver = accuracy[0] * can_gain;
	int16_t can_hor = accuracy[1] * can_gain;
	now = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {// have to wait because can message must transmit in time
		if ((HAL_GetTick() - now) > 4U) {
			break;
		}
	}

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0) {
		TxData[3] = can_hor >> 8U;
		TxData[2] = can_hor;
		TxData[1] = can_ver >> 8U;
		TxData[0] = can_ver;
		TxMSG.StdId = CAN_LOG_GPS_ID;  // number 8
		HAL_CAN_AddTxMessage(&hcan1, &TxMSG, TxData, &TxMailbox);
	}
}
