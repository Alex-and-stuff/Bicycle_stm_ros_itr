/*
 *
 *  Created on: 2019¦~12¤ë22¤é
 *      Author: LDSCB
 */

#include "controller.h"
/*
//#define DEBUGCONTROL 1

//RobustLMI = [65.5355   13.7367   -3.1852];
//RALMI = [350.3111   73.2225  -20.3968];
//KL = [267.7633   57.8611   -5.8690];
//RobustLMI[3] = {63.704,12.87,-5.569};
//RobustLMI[3] = {65.7986,13.7402,-6.0795}; for a = 0.265
float RobustLMI[3] = {5.5696,0.1006,3.75};	// K gain
float state_Robust[3] = {0,0,0};
float x_Robust[3] = {0,0,0};
uint32_t newstate_time = 0;
uint32_t oldstate_time = 0;
uint32_t newmotor_time = 0;
uint32_t oldmotor_time = 0;
uint32_t newcol_delay = 0;
uint32_t oldcol_delay = 0;
uint8_t motor_moving;

float based_goal = MXDEC_CENTER;	// 0x844 based on motor
float lowpass_fre=0;
float input_u = 0;
float input_oldu;
float lowpass_u = 0;

float output_delta = 0;
float last_output_delta = 0;
float last_last_output_delta = 0;
float notch_delta = 0;
float last_notch_delta = 0;
float last_last_notch_delta = 0;

float delta_refold = MXDEC_CENTER;
uint16_t hex_goal=0;
uint16_t MxUpper_bound = MXHEX_CENTER+0x400;
uint16_t MxLower_bound = MXHEX_CENTER-0x400;

float theta_error = 1.3;


void control(void){

//	V_current = 2.0;
//	if(V_current < V_STD){
//		V_current = V_REF;
//	}

//	float A[9]={0,1,0,
//			    (GRAVITY/length),0,0,
//				0,0,(-(V_current/a)-K_GAIN*(V_REF-V_current))
//	};

	float A[9]={0,1,0,
			    (GRAVITY/length),0,0,
				0,0,-(V_current/a)
	};

	float B[3] = {0, a*sin(epsilon)*(V_current/(b*length)) , 1};


#ifdef STATESAMPLE
	newstate_time = HAL_GetTick();
	uint16_t stadt = newstate_time - oldstate_time;
	if(stadt >= STATESAMPLE){
		volatile extern float Theta;		//extern
		volatile extern float Theta_dot;	//extern

		int delta_refu16;
		delta_refu16 = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);/// Read present position uint16_t
		float delta_ref = based_goal-delta_refu16*MXHEX2DEC;	//0.088 is 0~4096 map to 0~180 degree
//		float delta_ref = based_goal-hex_goal*MXHEX2DEC;

		x_Robust[0] = Theta*180/PI+theta_error;		// unit is rad because sensor -0.7 is balance
		x_Robust[1] = Theta_dot*180/PI;	// right is negtive left is positive from bicycle backward
		x_Robust[2] = delta_ref;
		oldstate_time = HAL_GetTick();
	}
#else
	volatile extern float Theta_y;		//extern
	volatile extern float Theta_ydot;	//extern
	int delta_refu16_tmp =0;
	float delta_ref; 	//0.088 is 0~4096 map to 0~180 degree
	uint8_t position_state;
	delta_refu16_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
	position_state = dxl_get_result();
	if(position_state == COMM_RXSUCCESS){
		delta_ref = (float)(MXHEX_CENTER - delta_refu16_tmp)*MXHEX2DEC;
	}else{
		delta_ref = delta_refold;
	}
	delta_refold = delta_ref;
//	delta_refu16 = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);/// Read present position uint16_t

//	float delta_ref = based_goal-hex_goal*MXHEX2DEC;

	x_Robust[0] = Theta_y*(180/PI)+theta_error;		// unit is rad because sensor -0.7 is balance
	x_Robust[1] = Theta_ydot*(180/PI);	// right is negtive left is positive from bicycle backward
	x_Robust[2] = delta_ref;
#endif


	float BK[9]={0};
	for(int i=0;i<3;++i){
		for(int j=0;j<3;++j){
			BK[i*3+j] = B[i]*RobustLMI[j];
		}
	}

	float A_BK[9]={0};
	for(int i=0;i<9;++i){
		A_BK[i] = A[i]+BK[i];
	}

	state_Robust[0] = 0;
	state_Robust[1] = 0;
	state_Robust[2] = 0;
	for(int i=0;i<3;++i){
		for(int j=0;j<3;++j){
			state_Robust[i] = state_Robust[i]+A_BK[i*3+j]*x_Robust[j];
		}
	}

	for(int i=0;i<3;++i){
		x_Robust[i] = x_Robust[i]+state_Robust[i]*SAMPLE_TIME*COLSAMPLE;	//dt 1000 is one second
	}

	input_u = 0;
	for(int i=0;i<3;++i){
		input_u = input_u + RobustLMI[i]*x_Robust[i];
	}

	input_u = -input_u;

	lowpass_fre = (V_current/a);			// delta = {(alpha/(s+alpha))*u}*(1/alpha) , alpha is lowpass_frequency
																//lowpass_fre = (V_current/a)+K*(V_REF-V_current);
	lowpass_u = low_pass(input_u,lowpass_u, lowpass_fre);

	output_delta = lowpass_u/lowpass_fre;
	notch_delta = notch_filter(last_notch_delta,last_last_notch_delta,output_delta,last_output_delta,last_last_output_delta);
	last_last_notch_delta = last_notch_delta;
	last_notch_delta = notch_delta;
	last_last_output_delta = last_output_delta;
	last_output_delta = output_delta;
	hex_goal = (based_goal-output_delta)/MXHEX2DEC;			// left small


#ifdef MOTORSAMPLE
	newmotor_time = HAL_GetTick();
	uint16_t motdt = newmotor_time - oldmotor_time;
	if(motdt >= MOTORSAMPLE){
		uint16_t colde = newcol_delay - oldcol_delay;
		if(colde >= COLDELAY){
			if(hex_goal <MXUPPER_BOUND && hex_goal > MXLOWER_BOUND){
//				motor_moving = dxl_read_byte(MOTOR_ID, P_MOVING); //dxl_read_byte(MOTOR_ID, P_MOVING)
//				uint8_t CommStatus = dxl_get_result();;//dxl_get_result();// test
//				if (CommStatus == COMM_RXSUCCESS){
//					if(motor_moving == 0){
						dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hex_goal);	//it is position moving
//							CW is small CCW is large (look to motor from forward which is connect to bicycle) range is 0x400 to 0xc00
//					}
//				}

			}
		}
//		}else{
//			hex_goal = based_goal;
//			if(newcol_delay == 0){
//				oldcol_delay = HAL_GetTick();
//			}
//			newcol_delay = HAL_GetTick();		// always large than oldcol_delay
//		}
		oldmotor_time = HAL_GetTick();
	}
#else
	if(hex_goal <MXUPPER_BOUND && hex_goal > MXLOWER_BOUND){
		dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hex_goal);
	}
#endif




#ifdef DEBUGCONTROL
//	int cc = x_Robust[0]*1000;
//	int cf = x_Robust[1]*1000;
//	int dd = x_Robust[2]*1000;
//	int jj = output_delta*1000;
//	int cc = A_BK[6]*1000;
//	int cf = A_BK[7]*1000;
//	int dd = A_BK[8]*1000;
//	printf("%5d,%5d,%5d\r\n",cc,cf,dd);
//		int jj = a*sin(epsilon)*(V_current/(b*length))*1000;
//		int vv = (based_goal-average_delta)*10;
		int ff = output_delta*1000;
//		int fl = hex_goal*1000;
//		int hh =(based_goal-output_delta)*10;//(based_goal-average_delta)*10;
////		printf("%5d,%5d,%5d\r\n",jj,vv,ff);
//		printf("%d,%d,\r\n",vv,hh);
	printf("%d\r\n",ff);
#endif
}


float low_pass(float input,  float output_old, float frequency){
	float output = 0;
	output = (output_old+frequency*SAMPLE_TIME*5*COLSAMPLE*input)/(1+frequency*SAMPLE_TIME*5*COLSAMPLE);
	return output;
}

float notch_filter(float last_notch_delta, float last_last_notch_delta, float output_delta, float last_output_delta, float last_last_output_delta){
	float output = 0;
	output = 1.9505*last_notch_delta-0.9512*last_last_notch_delta+output_delta-1.9944*last_output_delta+0.9951*last_last_output_delta;
	// s^2+0.5s+4/s^2+5s+4 matlab c2d method = 'zoh'
	return output;
}

*/
