/*
 * PDcontrol.c
 *
 *  Created on: 2020�~2��7��
 *      Author: LDSCB
 */

#include "controller.h"
#include "can.h"
#include "MxMotor.h"
#include "math.h"

extern float V_current;  // @TODO bad habit of using global variables

void PDD(float* state, float v, float* tracking_cmd, float remote_cmd, uint8_t distance_flag){

#ifdef NOSPEEDMODE
	v = 1.6;
#endif

	float Kp = -5.60f;
	float Kd = -0.28f;
	float dt = 0.01;
	float b = 1.053;
	float diff = 0.012f;

	float theta = state[0];
	float theta_dot = state[1];

	// Process remote control command from turn readings to randians
	uint16_t top = REMOTE_CENTER * 1.5;
	uint16_t dow = REMOTE_CENTER * 0.5;

	float vd = tracking_cmd[0];
	float wd = -tracking_cmd[1];
//	wd = -0.0;
	float rd = remote_cmd;
	vd = vd > V_MAX ? V_MAX : vd;
	vd = vd < V_MIN ? V_MIN : vd;
	wd = wd > W_MAX ? W_MAX : wd;
	wd = wd < W_MIN ? W_MIN : wd;
	rd = rd > top ? top : rd;
	rd = rd < dow ? dow : rd;

	float CMD_DELTA_MAX = 0.436332f;
	float CMD_DELTA_MIN = -0.436332f;
	float remote_delta = CMD_DELTA_MAX * (REMOTE_CENTER - rd)/(top - REMOTE_CENTER);

	// ========== test =================
	// Process tracking command from angular velocity to radians
	float tracking_delta = (atan(b * wd / v) / sin(epsilon));  // vd changed to v for convenience
	if (fabs(tracking_delta) > CMD_DELTA_MAX) {
		if (tracking_delta > 0)
			tracking_delta = CMD_DELTA_MAX;
		else
			tracking_delta = -CMD_DELTA_MAX;
	}

	float delta_d = tracking_delta + remote_delta;
	float delta_d_lp = delta_d;
	static float delta_d_lp_old = 0.0f;
	delta_d_lp = LowpassFilter(delta_d_lp, delta_d_lp_old, 5.0f, dt); // changed from 4.0f to 5.0f
	delta_d_lp_old = delta_d_lp;

	delta_d = delta_d_lp;
	delta_d = delta_d > CMD_DELTA_MAX ? CMD_DELTA_MAX : delta_d;
	delta_d = delta_d < CMD_DELTA_MIN ? CMD_DELTA_MIN : delta_d;
    //=================================

	float delta_control = theta * Kp + theta_dot * Kd;

	#if TESTEKF == 1
		tracking_delta = 0.0f;
	#endif

	float tracking_omega_max = 0.436332f; //delta 25(degree)
	float tracking_omega_min = -0.436332f; //delta 25(degree)

	static float command_old = 0.0f;
	float command = delta_d; // + remote_delta_lp; // tracking delta + remote command delta
//	*tracking_cmd = (atan(1.053 * tracking_delta / v) / sin(epsilon));	//L : bicycle  length = 1.053(meter)

	if (command > tracking_omega_max) {
			command = tracking_omega_max;
	}
	if(command < tracking_omega_min) {
			command = tracking_omega_min;
	}

//	if (distance_flag) {
//
//		float init_bound = 0.15f;
//		if (fabsf(command) > init_bound) {
//			command = command > 0 ? init_bound : -init_bound;
//		}
//	}

	if (command - command_old > diff) {// avoid omega different large
			command = command_old + diff;
	}
	if (command - command_old < -diff) {
			command = command_old - diff;
	}
	command_old = command;


//	int16_t tracking_cmd_hex = (delta_d + remote_delta_lp) * RAD2DEG / MXHEX2DEC;
	int16_t tracking_cmd_hex = (command) * RAD2DEG / MXHEX2DEC;

	uint16_t hexgoal = (uint16_t) ((MXDEC_CENTER - delta_control*180.0f/PI) / MXHEX2DEC + tracking_cmd_hex);
	if (hexgoal < MXUPPER_BOUND && hexgoal > MXLOWER_BOUND) {
		dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hexgoal);
	}

}
void PDControl(float theta, float theta_dot, float tracking_angle, float *remote, float *delta_output, uint8_t distance_flag, float bias) {
/* @input: theta (bicycle roll angle), theta_dot, omega_command, remote, delta_output, distance_flag
 * @note: *remote gives back the remote position (rad)?
 * @note: *delta_output gives back the delta command angle (rad)?
 *
 **/

//	////////////////////////////////////// Parameter Control
//
	float Kp = -5.6f;			//2020_1105 5.6 //2020_0419 5.5   5.365  2.8(m/s):3.23			//3.262		-5.6
	float Kd = -0.28f;			//2020_1105 0.09//2020_0419 0.09
	uint16_t remote_gain = 2;		// is is bigger the remote control is smaller
//
//	////////////////////////////////////// Parameter Control

	uint16_t RX1 = REMOTE_CENTER;
	uint16_t RX2 = REMOTE_CENTER;
	uint16_t RX3 = REMOTE_LOWER;
#ifdef REMOTE_CONTROL
	start_read(&RX1, &RX2, &RX3);
	uint16_t top = 1861;
	uint16_t dow = 1179;
	RX1 = RX1 > REMOTE_CENTER * 1.1 ? top : RX1;
	RX1 = RX1 < REMOTE_CENTER * 0.9 ? dow : RX1;
#endif

//	float delta_ref = MXDEC_CENTER;
//	float Mxspeed_rad = 0.0f;

//	float delta_ref = ReadDeltaAngle();
//	float Mxspeed_rad = ReadDeltaOmega();

	float state_compensation = bias;
//	float theta_y = (theta + state_compensation * delta_ref + THETA_COM * (PI / 180.0f));				//because IMU position change so the variable is y
//	float theta_ydot = (theta_dot + state_compensation * Mxspeed_rad);	//negative feedback
	float theta_y = (theta + state_compensation);				//because IMU position change so the variable is y
	float theta_ydot = (theta_dot + state_compensation);	//negative feedback

	float delta_control = 0;
//	delta_control = theta_y * Kp / (1 + Kp * Ku) + theta_ydot * Kd / (1 + Kd * Ku); //unit rad
	delta_control = theta_y * Kp + theta_ydot * Kd;

	float dt = SAMPLE_TIME * COLSAMPLE;
	static uint16_t RX1_last = REMOTE_CENTER;

#ifdef REMOTE_CONTROL
	if (RX1 < REMOTE_LOWER * (1 - REMOTE_ERROR)|| RX1 > REMOTE_UPPER * (1 + REMOTE_ERROR)) { // 10% can afford error ,to deal with no remote data
		RX1 = REMOTE_CENTER;	// mean no control
	} else {
		RX1 = LowpassFilter(RX1, RX1_last, REMOTE_LOWPASS_FREQUENCY, dt);
		RX1_last = RX1;
	}
#endif
	//@TODO
//	printf("tkkk %.3\r\n",tracking_angle);
#if TESTEKF == 1
	tracking_angle = 0.0f;
#endif
	/////////////////////////////////////////////////////////// for protect

	float tracking_omega_max = 0.436332f; //delta 25(degree)
	float magic_number = 10.0f;
	float diff = 0.02f;

	tracking_angle = tracking_angle/magic_number;

	static float tracking_cmd_old = 0.0f;
	float tracking_cmd = 0;
	tracking_cmd = (atan(1.053 * tracking_angle / V_current) / sin(epsilon));	//L : bicycle  length = 1.053(meter)
	//delta_d = atan(phi_dot*L)/sin(epsilon), tracking_angle = omega = phi_dot
	if (fabs(tracking_cmd) > tracking_omega_max) {
		if (tracking_cmd > 0)
			tracking_cmd = tracking_omega_max;
		else
			tracking_cmd = -tracking_omega_max;
	}

	if (distance_flag) {
		//tracking_cmd = LowpassFilter(tracking_cmd, tracking_cmd_old, 1.0f, dt);	// if distance is large control rate need small
		float init_bound = 0.15f;
		if (fabsf(tracking_cmd) > init_bound) {
			tracking_cmd = tracking_cmd > 0 ? init_bound : -init_bound;
		}
	}

	if (fabsf(tracking_cmd - tracking_cmd_old) > diff) {// avoid omega different large
		if ((tracking_cmd - tracking_cmd_old) > 0)
			tracking_cmd = tracking_cmd_old + diff;
		else
			tracking_cmd = tracking_cmd_old - diff;
	}
	tracking_cmd_old = tracking_cmd;
	///////////////////////////////////////////////////////////

	int16_t tracking_cmd_hex = tracking_cmd * RAD2DEG / MXHEX2DEC;
	int16_t remote_angle = (REMOTE_CENTER - RX1) / remote_gain;//angle bigger let bicycle turn left // divide 2 because limit angle
	*remote = remote_angle*MXHEX2DEC*DEG2RAD+tracking_cmd;
	*delta_output = delta_control + *remote;
	uint16_t hexgoal = MXHEX_CENTER;
	hexgoal = (uint16_t) ((MXDEC_CENTER - delta_control*180.0f/PI) / MXHEX2DEC + remote_angle + tracking_cmd_hex);
	if (hexgoal < MXUPPER_BOUND && hexgoal > MXLOWER_BOUND) {
		dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hexgoal);
	}
//	int aa = delta_control*180.0f/PI*1000;
//	printf("%d,",aa);
//	printf("%.3f,%.3f\r\n", tracking_cmd, *delta_output);
//	printf("= %4.3f %4.3f %4.3f\r\n",*delta_output, *remote ,delta_control);
//	printf("%.3f\r\n", delta_control);
}

float LowpassFilter(float input, float output_old, float frequency, float dt) {
	float output = 0;
	output = (output_old + frequency * dt * input) / (1 + frequency * dt);
	return output;
}

float ReadDeltaAngle(){
	int delta_refu16_tmp = 0;		// delta angle type uint16_t
	float delta_ref = MXDEC_CENTER;
	uint8_t position_state;
	delta_refu16_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
	position_state = dxl_get_result();
	if (position_state == COMM_RXSUCCESS) {
		if (delta_refu16_tmp < MXUPPER_BOUND && delta_refu16_tmp > MXLOWER_BOUND) {
			delta_ref = (float) (MXHEX_CENTER - delta_refu16_tmp) * MXHEX2DEC * DEG2RAD;			// get the delta angle unit(rad)
		}
	}

	return delta_ref;
}

float ReadDeltaOmega(){
	int16_t Mxspeed = 0;		// Mx106 motor speed
	int16_t Mxspeed_tmp = 0;
	uint8_t rs485_state = 0;
	Mxspeed_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_SPEED_L);
	rs485_state = dxl_get_result();
	if (rs485_state == COMM_RXSUCCESS) {
		if (Mxspeed_tmp >= SPEED_THRESHOLD) {
			Mxspeed = -(Mxspeed_tmp - SPEED_THRESHOLD);
		} else {
			Mxspeed = Mxspeed_tmp;
		}
	}
	float Mxspeed_rad = -(float)Mxspeed * MXSPD_HEX2RPM * 2.0f * PI / 60.0f;

	return Mxspeed_rad;
}

//void Remote_read(void) {
//Capture RX signal as PWM
//RXprd = LL_TIM_IC_GetCaptureCH1(TIM8);
//	RX1 = LL_TIM_IC_GetCaptureCH1(TIM8);	// *this is correct to read CH1
//	RX2 = LL_TIM_IC_GetCaptureCH3(TIM8);
//	RX3 = LL_TIM_IC_GetCaptureCH4(TIM8);

//	//Capture out off range
//	if ((RX3 > 2200U) || (RX3 < 800U)) {
//		FAULT |= RXVAL_FaultCode;
//	}
//
//	//Switch RX mode
//	if (RX3 > 1500U) {
//		RX_mode = 1U;
//	} else {
//		RX_mode = 0U;
//	}
//}
