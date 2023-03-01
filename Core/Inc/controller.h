/*
 * controller.h
 *
 *  Created on: 2020�~7��6��
 *      Author: ldsc8
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_




#include "sensor_fusion.h"

#define REMOTE_LOWPASS_FREQUENCY	4.0f

void control(void);
float low_pass(float input,  float output_old, float frequency);
void PDControl(float theta, float theta_dot, float tracking_angle, float *remote_angle, float *hexgoal,uint8_t distance_flag, float bias);
float notch_filter(float last_notch_delta, float last_last_notch_delta, float output_delta, float last_output_delta, float last_last_output_delta);
float LowpassFilter(float input,  float output_old, float frequency, float dt);
void LMI(float theta, float theta_dot, float tracking_omega, float *remote_angle, float *delta_output, float vel);
void LMII(float theta, float theta_dot, float tracking_vd, float tracking_wd, float *remote_angle, float *delta_output, float vel, float distance_flag, float bias);
void LMIII(float* state, float v, float* tracking_cmd, float remote_cmd);
void controldesign(float theta_x, float gyrox, float tracking_omega, float *remote_angle, float *delta_output, float vel);
void estimator(float theta, float theta_dot, float *bias);

/////////////////////////////////////////////// bicycle constant
//float length = 0.4338;
//float a = 0.3957;
//float b = 1.053;
//float K = 0.6223;		// is from senior
//float epsilon = 1.22173; //degree 70
/////////////////////////////////////////////// bicycle constant

#endif /* INC_CONTROLLER_H_ */

