#include "controller.h"
#include "MxMotor.h"
#include "math.h"

void estimator(float theta, float theta_dot, float *bias){
	static float count = 0.0f;
	if(count<1000000){count=count+1.0f;}
	// Get MX motor delta data
	int delta_refu16_tmp = 0;
	static float delta_ref = MXHEX_CENTER* PI / 180.0f;  // 2022.8.17 changed to static float
	uint8_t position_state;
	delta_refu16_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
	position_state = dxl_get_result();
	if (position_state == COMM_RXSUCCESS) {
		if (delta_refu16_tmp < MXUPPER_BOUND && delta_refu16_tmp > MXLOWER_BOUND) {
			delta_ref = (float) (MXHEX_CENTER - delta_refu16_tmp) * MXHEX2DEC * DEG2RAD;
		}
	}

	// Design an observer to calculate the IMU theta bias
	float K[3] = {25.8319,    6.6856,   -0.8369};
	float x[3] = {theta, theta_dot, delta_ref};
	float u_init = 0;

	for(int i = 0; i < 3; i ++){
		u_init -= K[i]*x[i];
	}

//	float A[] = {1.0009,    0.0100,         0,      0,
//				 0.1852,    1.0009,         0,      0,
//					  0,         0, expf(-2*2/79),  0,
//					  0,         0,         0,      1};
//
//	float B[] = {0.0001, 0.0133, -(79*(expf(-2*2/79)-1)/200/2), 0};
	float A[] = {1.0009,    0.0100,         0,      0,
				 0.1852,    1.0009,         0,      0,
					  0,         0,    0.9506,      0,
					  0,         0,         0,      1};

	float B[] = {0.0001, 0.0133, 0.0098, 0};

	float C[] = {1,0,0,1,
				 0,1,0,0,
				 0,0,1,0};

	float L[] = {1.1208,    9.0902,         0,
				 0.1587,    2.6012,         0,
					  0,         0,    1.0506,
				 0.1799,   -9.0729,         0};

	static float xk[4] = {0};
	float xk_new[3] = {0};
	float measure_err[3] = {theta, theta_dot, delta_ref};
	static float accum = 0;

	for(int i = 0; i < 4; i++){
		xk_new[i] = A[4*i]*xk[0] + A[4*i+1]*xk[1] + A[4*i+2]*xk[2] + A[4*i+3]*xk[3];
	}
	for(int i = 0; i < 4; i++){
		xk_new[i] += B[i]*(u_init); //B*u, u must be positive
	}
	for(int i = 0; i < 3; i++){
		measure_err[i] -= (C[4*i+0]*xk[0]+C[4*i+1]*xk[1]+C[4*i+2]*xk[2]+C[4*i+3]*xk[3]);
	}
	for(int i = 0; i < 4; i++){
		xk_new[i] += L[3*i]*measure_err[0]+L[3*i+1]*measure_err[1]+L[3*i+2]*measure_err[2];
		xk[i] = xk_new[i];
	}

	if(count<1000000){
		accum += xk_new[3];
	}
	*bias = accum/count;
//	printf("%.3f, %.3f, %.3f, %.3f\r\n", theta, theta_dot, delta_ref, u_init);
//	printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.1f\r\n", xk_new[0], xk_new[1], xk_new[2], xk_new[3], *bias, count);
//	printf("%.3f\r\n", *bias);
}
