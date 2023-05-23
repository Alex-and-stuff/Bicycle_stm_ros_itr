#include "controller.h"
#include "MxMotor.h"
#include "math.h"
#include "can.h"

float a = 0.3;	// bicycle parameter
float b = 1.053;
float dt = 0.01;

#define PD  0

float DeltaCmdControl(float delta_output, float delta_cmd);
void LMIII(float* state, float v, float* tracking_cmd, float remote_cmd){

#ifdef NOSPEEDMODE
	v = 1.6;
#endif

    float theta = state[0];
    float theta_dot = state[1];
    float delta = state[2];

    // Process remote control command from turn readings to randians
    uint16_t top = REMOTE_CENTER * 1.5;
	uint16_t dow = REMOTE_CENTER * 0.5;

    float vd = tracking_cmd[0];
    float wd = -tracking_cmd[1];
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
    float remote_delta_lp = remote_delta;
	static float remote_delta_lp_old = 0.0f;
	remote_delta_lp = LowpassFilter(remote_delta_lp, remote_delta_lp_old, 4.0f, dt);
	remote_delta_lp_old = remote_delta_lp;

    // Process tracking command from angular velocity to radians
    float tracking_delta = (atan(b * wd / vd) / sin(epsilon));
    if (fabs(tracking_delta) > CMD_DELTA_MAX) {
		if (tracking_delta > 0)
			tracking_delta = CMD_DELTA_MAX;
		else
			tracking_delta = -CMD_DELTA_MAX;
	}

    // @TODO

    // Merge controls from remote and tracking
    float delta_d = tracking_delta + remote_delta;
    delta_d = delta_d > CMD_DELTA_MAX ? CMD_DELTA_MAX : delta_d;
    delta_d = delta_d < CMD_DELTA_MIN ? CMD_DELTA_MIN : delta_d;

    delta_d = delta_d/100; ////////////////

    // Start LMI control when vehicle has achieved a specific velocity
    if(v > V_STD){
//        float K[]   = {32.4522f, 7.0690f, 0.0179f/100.0f};

//    	float K[3]   = {24.3391,    6.1548,    0.00524}; // 4 1.5 2.3 221 a = 10
//    	float K[3]   = {23.8200,    6.1171,    0.0896};  // 4 1.5 2.2 221 a = 10
//    	float K[3]   = {32.8939,    6.5065,   -1.0786};  // 4 1.5 1.2 12  a = 1

//        float K[]   = {0.3376,    0.0686,   -0.0004};  // 4 1.5 0.8 15 a:100  can somewhat work
        float K[]   = {0.3930,    0.1248,   -0.0006};  // 4 1.5 0.8 32.5 a:100 no good
//        float K[]   = { 0.3677,    0.1829,   -0.0007}; // 4 1.5 0.2 51.5 a:100 no good

        float x[3]   = {theta + THETA_COM*PI/180.0f, theta_dot, delta};
//    	float x[3]   = {theta, theta_dot, delta};
        float x_d[3] = {vd*vd*sinf(epsilon)/(-GRAVITY*b)*delta_d, 0, delta_d};
        float u_d    = (vd/a) * delta_d;
        float u_bar  = 0.0f;
        float u      = 0.0f;

        for(int i = 0; i < 3; i++){
            x[i] = x[i] - x_d[i];
        }

        for(int i = 0; i < 3; i++){
            u -= K[i] * x[i];
        }
        u_bar = u + u_d;

        u_bar = u_bar*100; ////////////////

        // C2d delta/u transfer function
        static float output_old = 0.0f;
        const float sample_time = 0.01f;
        const float Ka = 2.54f;
        float acc_term = Ka * (vd - v);
        float output = (output_old+sample_time*u_bar) / (1+v/a*sample_time+acc_term*sample_time);
        output_old = output;


        float OUTPUT_MAX_RAD = 70 * PI / 180;  // ~0.349 ~0.523
		if (output > OUTPUT_MAX_RAD){ output = OUTPUT_MAX_RAD;}
		if (output < -OUTPUT_MAX_RAD){ output = -OUTPUT_MAX_RAD;}

        uint16_t hexgoal = (uint16_t)((MXDEC_CENTER-output*180.0f/PI) / MXHEX2DEC);
#ifdef INIT_MX
        if(hexgoal < MXUPPER_BOUND && hexgoal > MXLOWER_BOUND){
            dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hexgoal);
        }
#endif
    }
}

void LMII(float theta, float theta_dot, float tracking_vd, float tracking_wd, float *remote_angle, float *delta_output, float v0, float distance_flag, float bias) {
	// New version of LMI written in 2022.8.4.
	// - Removed most of the magic numbers from the old version
	// - Seems to work in both remote control mode and tracking mode (not tested yet)

	if(tracking_vd>2.5){tracking_vd = 2.5f;}
	if(tracking_vd<1.5){tracking_vd = 1.5f;}
	/* =====================================
	 * ===== Remote control parameters =====
	 * ===================================== */
	uint16_t RX1 = REMOTE_CENTER;
	uint16_t RX2 = REMOTE_CENTER;
	uint16_t RX3 = REMOTE_LOWER;
	uint16_t top = REMOTE_CENTER * 1.2;
	uint16_t dow = REMOTE_CENTER * 0.8;
#ifdef REMOTE_CONTROL
	start_read(&RX1, &RX2, &RX3);
	RX1 = RX1 > top ? top : RX1;
	RX1 = RX1 < dow ? dow : RX1;
#endif
	// Remote output ranged between (+/-) 15 degrees then converted to radians
	float MX_TURN_LMT_OMEGA = 0.5f;
    float omega_remote = MX_TURN_LMT_OMEGA * (float)(REMOTE_CENTER - RX1)/(float)(top - REMOTE_CENTER); /////////

    // Add low pass filter to remote output
	float omega_remote_lp = omega_remote;
	static float omega_remote_lp_old = 0.0f;
	omega_remote_lp = LowpassFilter(omega_remote_lp, omega_remote_lp_old, 4.0f, dt);	//if tracking omega need lowpass
	omega_remote_lp_old = omega_remote_lp;

	/* =====================================
	 * ==== Tracking control parameters ====
	 * ===================================== */
	float tracking_delta_max = 0.436332f; // unit: radian, 25(degree)
	float magic_number = 10.0f; // @TODO might want to lower the gain in tracking controldelta_d
	float diff = 0.02f; // unit: delta rad, ~.15(degree)
	float delta_tracking = 0.0f;
	static float delta_tracking_old = 0.0f;

	tracking_wd = tracking_wd/magic_number;


	delta_tracking = (atan(b * (tracking_wd) / tracking_vd) / sin(epsilon));
	if(tracking_vd == 0) delta_tracking = 0;
	// Limit tracking angle to around 25 deg
	if (fabs(delta_tracking) > tracking_delta_max) {
		if (delta_tracking > 0)
			delta_tracking = tracking_delta_max;
		else
			delta_tracking = -tracking_delta_max;
	}

	// If current state too far from the virtual bicycle
	if (distance_flag) {
		float init_bound = 0.15f;
		if (fabsf(delta_tracking) > init_bound) {
			delta_tracking = delta_tracking > 0 ? init_bound : -init_bound;
		}
	}
	// Avoid delta different large
	if (fabsf(delta_tracking - delta_tracking_old) > diff) {
		if ((delta_tracking - delta_tracking_old) > 0)
			delta_tracking = delta_tracking_old + diff;
		else
			delta_tracking = delta_tracking_old - diff;
	}
	delta_tracking_old = delta_tracking;


//#ifndef EKF
//	tracking_vd = 2.0f;
//	tracking_wd = tracking_vd*tan(omega_remote_lp*sin(epsilon))/b; // should be remote control wd, not 0
//#endif

	/* =====================================
	 * ====== Convert w_d to delta_d  ======
	 * ===================================== */
	float delta_d = 0.0f;
#ifdef REMOTE_CONTROL
	tracking_vd = 2.0f;
	delta_d = atan(b*omega_remote_lp/tracking_vd)/sin(epsilon);
#endif
#ifndef REMOTE_CONTROL
	delta_d = delta_tracking;
#endif
    // Read steering angle from MX motor (radian)
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

	// Lowpass theta_dot results
//	float theta_dot_lp = theta_dot;
//	static float theta_dot_lp_old = 0.0f;
//	theta_dot_lp = LowpassFilter(theta_dot_lp, theta_dot_lp_old, 8.0f, dt);	//if tracking omega need lowpass
//	theta_dot_lp_old = theta_dot_lp;
	float theta_dot_lp = theta_dot;

	if (v0 > V_STD){

		// Generate the states x and x_d
//		float K[3] = {266.1011,  43.6233  , -17.3143};

//		float K[3] = {21.4846,    4.6387,   -1.7916}; // 2.5 1.5 0.5 5 damping 2
//		float K[3] = {26.6142,    5.2750,   -1.3401}; //damping
//		float K[3] = {17.6068,    4.2238,   -1.8722}; // 2.5 1.5 0.3 4.4 bad
//		float K[3] = {28.2714,    6.1032,   -1.0096}; // 4 1.5 0.8 15 small damp
//		float K[3] = {27.1966,    5.9315,   -1.0198}; // 4 1.5 0.6 15 not tested
//		float K[3] = {24.7064,    5.0681,   -0.9585}; // 4 1.5 0.5 12 pretty good but still visible damping
//		float K[3] = {22.6460,    4.5758,   -0.9733}; // 4 1.5 0.2 11 pretty good but still visible damping
//		float K[3] = {24.0147,    5.3256,   -0.9168}; // 4 1.5 0.2 14 pretty good but still visible damping
//		float K[3] = {25.9996,    5.5605,   -1.0357}; // 4 1.5 0.5 14 no good
//		float K[3] = {24.1190,    5.4791,   -0.9541}; // 4 1.5 0.1 15 better? i guess
//		float K[3] = {25.5548,    6.1997,   -0.9412}; // 4 1.5 0.1 18 even better? (when imu good)
//		float K[3] = {25.7067,    6.5788,   -0.8584}; // 4 1.5 0.1 20 almost there ****GOOD!!!!!***
//		float K[3] = {27.7095,    7.1993,   -0.8526}; // 4 1.5 0.1 22
//		float K[3] = {27.0485,    6.8698,   -0.9005}; // 4 1.5 0.1 20.6
//		float K[3] = {25.8319,    6.6856,   -0.8369}; // 4 1.5 0.1 20.5 best
//		float K[3] = {24.1756,    5.2363,   -0.8405}; // 4 1.5 0.1 20.5 h=0.4 bad
//		float K[3] = {26.2720,    5.9312,   -0.6982}; // 4 1.5 1.5 10 h=0.5 a=0.3  best=-0.1982
//		float K[3] = {25.1229,    5.2253,   -1.6359}; // useable
//		float K[3] = {29.3287,    5.9947,   -1.4941};// 4 1.5 0.5 12
//		float K[3] = {32.4522,    7.0690,   -1.6124}; // good gain!!!!
		float K[3] = {32.4522,    7.0690,   0.0179/100};


//		float x[3] = {theta + THETA_COM * PI / 180.0f, theta_dot_lp, delta_ref};  // 2022.8.17 added THETA_COM to the first term
		float x[3] = {theta + THETA_COM * PI / 180.0f, theta_dot_lp, delta_ref};  // 2022.8.17 added THETA_COM to the first term
		float x_d[3] = {tracking_vd * tracking_vd * sinf(epsilon) / (-GRAVITY * b) * delta_d, 0, delta_d};//+delta_ref}; // here I think vd should be used instead of v0?
		float u_d = (tracking_vd / a) * delta_d;
		float u_bar = 0.0f;
		float u = 0.0f;
//		printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", x[0], x[1], x[2], x_d[0], x_d[1], x_d[2]);
		// Compute x_bar
//		x[0] += bias;
		for(int i = 0; i < 3; i ++){
			x[i] = x[i] - x_d[i];
		}
		// Compute steering control u_bar. The computed u here is the angular velocity command, but the MX motor input is degrees (NEED CONVERSSION)
		for(int i = 0; i < 3; i ++){
			u -= K[i]*x[i];  // The MX motor is installed upside down (sign change? I think)
		}
		u_bar = u + u_d;

		/* =================================
		 * ==== c2d delta/u transfer fcn ===
		 * ================================= */
		static float output_old = 0.0f;
		float sample_time = 0.01f;
		float Ka = 2.54;
		float acc_term = Ka*(tracking_vd-v0);
		float output = (output_old+sample_time*u_bar)/(1+v0/a*sample_time+acc_term*sample_time);
//		float output = (output_old+sample_time*u_bar)/(1+v0/a*sample_time);

		output_old = output;

//		printf("%.3f, %.3f, %.3f\r\n",theta, x_d[0], x[0]);
//		printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",K[0]*x[0], K[1]*x[1], K[2]*x[2], x[0], x[1], x[2]);
//		printf("%.3f, %.3f, %.3f, %.3f\r\n",K[0]*x[0], K[1]*x[1], K[2]*x[2], output);
//		printf("%.3f, %.3f, %.3f\r\n", delta_ref,tracking_wd,delta_tracking);

		float delta_cmd = output;
		static float delta_cmd_old = 0;
		float DELTA_DIFF = 0.5; //0.1
		if(delta_cmd > (delta_cmd_old + DELTA_DIFF)){delta_cmd = delta_cmd_old + DELTA_DIFF;}
		if(delta_cmd < (delta_cmd_old - DELTA_DIFF)){delta_cmd= delta_cmd_old - DELTA_DIFF;}

		float OUTPUT_MAX_RAD = 70 * PI / 180;  // ~0.349 ~0.523
		if (delta_cmd > OUTPUT_MAX_RAD){ delta_cmd = OUTPUT_MAX_RAD;}
		if (delta_cmd < -OUTPUT_MAX_RAD){ delta_cmd = -OUTPUT_MAX_RAD;}

		delta_cmd_old = delta_cmd;

		*delta_output = delta_cmd;
//		printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n",tracking_vd,tracking_wd,delta_tracking,delta_cmd, u , u_d);

//		float output_lp = output;
//		static float output_lp_old = 0.0f;
//		output_lp = LowpassFilter(output_lp, output_lp_old, 5.0f, dt);	//if tracking omega need lowpass
//		output_lp_old = output_lp;

		// Output control to MX motor. Input from radians to degrees, then converted to hexadecimal
		uint16_t hexgoal = MXHEX_CENTER;
		// By multiplying control (u_bar) with the sampling rate, I think I can get the steering command in radians



		hexgoal = (uint16_t) ((MXDEC_CENTER - delta_cmd * 180.0f / PI) / MXHEX2DEC);
	#ifdef INIT_MX
		if ((hexgoal < MXUPPER_BOUND && hexgoal > MXLOWER_BOUND)) {
			dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hexgoal);
		}
	#endif
	}
}

void LMI(float theta, float theta_dot, float tracking_omega, float *remote_angle, float *delta_output, float vel) {
	uint16_t hexgoal = MXHEX_CENTER;
	float output = 0.0f;
	uint8_t vo_flag = 1;
#if PD == 0

	static float output_old = 0.0f;
	static float output_old_old = 0.0f;
	static float notch_old = 0.0f;
	static float notch_old_old = 0.0f;

	uint16_t RX1 = REMOTE_CENTER;
	uint16_t RX2 = REMOTE_CENTER;
	uint16_t RX3 = REMOTE_LOWER;

	float magic_number = 6.0f;	// the tracking gain is too large
	float dt = COLSAMPLE * SAMPLE_TIME;
//	float K[3] = { -800.1238 , -10.1508 , -14.3912}; // lqr 301.9823 ,  19.9858 ,   0.6734 , LMI -639.0131  ,-21.6701  ,-37.2576
//		float K[3] = {-534.7268, -23.3110, -53.3624 };
//		float K[3] = {-484.9029,  -44.8793 , -15.7613 };
//	float K[3] = {-208.5713,  -44.1755  , -3.7342}; // original
	float K[3] = {-266.1011,  -43.6233  , 17.3143}; //2022.7.25 changed
//	float K[3] = {284.7354, 58.2616, -35.8509};
	float tk_bound = 0.2f;	//omega_bounded
	float tk_diff_bound = 0.1f;

	int delta_refu16_tmp = 0;
	float delta_ref = MXHEX_CENTER* PI / 180.0f;
	uint8_t position_state;
	delta_refu16_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
	position_state = dxl_get_result();
	if (position_state == COMM_RXSUCCESS) {
		if (delta_refu16_tmp < MXUPPER_BOUND && delta_refu16_tmp > MXLOWER_BOUND) {
			delta_ref = (float) (MXHEX_CENTER - delta_refu16_tmp) * MXHEX2DEC * DEG2RAD;
		}
	}

	float v0 = vel;

	tracking_omega = tracking_omega/magic_number; //need to change bound
	static float tracking_omega_old = 0.0f;
	if(fabsf(tracking_omega - tracking_omega_old) > tk_diff_bound){		// ignore if omega command differential is large in original
		if(tracking_omega > 0.0f) tracking_omega = tracking_omega_old + tk_diff_bound;
		else tracking_omega = tracking_omega_old - tk_diff_bound;
	}
	tracking_omega_old = tracking_omega;	// constrain the omega differential

	float tracking_delta = atanf(b * (tracking_omega) / v0) / sinf(epsilon);
//	tracking_delta = tracking_delta / magic_number;
	if (fabsf(tracking_delta) > tk_bound) {
		tracking_delta = tracking_delta > 0 ? tk_bound : -tk_bound;
	}

//	start_read(&RX1, &RX, &RX);
	start_read(&RX1, &RX2, &RX3);
//	uint16_t top = 1633;
//	uint16_t dow = 1407;
	uint16_t top = 1861;
	uint16_t dow = 1179;
	RX1 = RX1 > REMOTE_CENTER * 1.1 ? top : RX1;
	RX1 = RX1 < REMOTE_CENTER * 0.9 ? dow : RX1;
	float delta_remote = -15.0f * PI / 180.0f * (RX1 - REMOTE_CENTER) / (REMOTE_UPPER - REMOTE_CENTER);
	static float delta_remote_old = 0.0f;
	delta_remote = LowpassFilter(delta_remote,delta_remote_old,REMOTE_LOWPASS_FREQUENCY,dt);
	delta_remote_old = delta_remote;
//	delta_remote = delta_remote > 5.0f*DEG2RAD?5.0f*DEG2RAD:0.0f;
//	printf("==> %.3f\r\n",delta_remote);
//	cde=delta_remote;

#if TESTEKF == 1
	tracking_delta = 0.0f;
#endif

//	static float tracking_delta_old = 0.0f;
//	tracking_delta = LowpassFilter(tracking_delta, tracking_delta_old, 2.0f, dt);	//if tracking omega need lowpass
//	tracking_delta_old = tracking_delta;

	float deltaD = delta_remote + tracking_delta;
	*remote_angle = deltaD;
//	float tmp_delta = DeltaCmdControl(*delta_output, deltaD);
//	deltaD = deltaD+tmp_delta;
//	deltaD = -deltaD;

	float x[3] = { theta + THETA_COM * PI / 180.0f, theta_dot, delta_ref};
	float xd[3] = { v0 * v0 * sinf(epsilon) / (-GRAVITY * b) * deltaD, 0, deltaD };
	for (int i = 0; i < 3; i++) {
		x[i] = x[i] - xd[i];  // 2022.7.25 changed to "-"
	}
	float input_u = 0;
	for (int i = 0; i < 3; ++i) {
		input_u = input_u + K[i] * x[i];
	}
	input_u = input_u + deltaD * (v0 / a);

	float frequency = v0 / a;
	if (v0 < V_STD) {
		vo_flag = 0;
	}

	if (vo_flag) {
		output = LowpassFilter(input_u, output_old, frequency, dt);
		output = output / frequency;

		float notch = notch_filter(notch_old,notch_old_old,output,output_old,output_old_old);
		output_old_old = output_old;
		output_old = output;

		notch_old_old = notch_old;
		notch_old = notch;

//		output = notch;//////////////////////////////////////??????????????????
//		int bb = notch * 180.0f / PI*1000;
//		printf("%d,%d\r\n",aa,bb);
//		int vvv = (int)(v0*1000);
//		printf("GOOD, %d\r\n", vvv);
//		printf("%d", aa);
	}
//	else{
//		int vvv = (int)(v0*1000);
//		printf("no vo_flag!!!!!, %d\r\n", vvv);
//	}

	*delta_output = output;
#endif
#if PD == 1
	float Kp = 5.6;
	float Kd = 0.09;
	float theta_y = theta;	//because IMU position change so the variable is y
	float theta_ydot = theta_dot;

	float delta_control = 0;
	delta_control = (theta_y * (180 / PI) + THETA_COM) * Kp + theta_ydot * (180 / PI) * Kd; //unit degree
	output = -delta_control*PI/180.0f;

#endif
	hexgoal = (uint16_t) ((MXDEC_CENTER - output * 180.0f / PI) / MXHEX2DEC);

#ifdef INIT_MX
	if ((hexgoal < MXUPPER_BOUND && hexgoal > MXLOWER_BOUND) && vo_flag) {
		dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hexgoal);
	}
//	printf("%.3f,%.3f\r\n",output, delta_ref);
//	int aa = output * 180.0f / PI*1000;
//    printf("%d\r\n",aa);
//	int ad = tracking_delta*1000;
//	printf("%d,%d\r\n",aa,ad);
#endif

}

//==================== New Version 2022.7.25 ==========================
//void LMI(float theta, float theta_dot, float tracking_omega, float *remote_angle, float *delta_output, float vel) {
//	uint16_t hexgoal = MXHEX_CENTER;
//	float output = 0.0f;
//	uint8_t vo_flag = 1;
//#if PD == 0
//
//	static float output_old = 0.0f;
//	static float output_old_old = 0.0f;
//	static float notch_old = 0.0f;
//	static float notch_old_old = 0.0f;
//	uint16_t RX1 = REMOTE_CENTER;
//	uint16_t RX2 = REMOTE_CENTER;
//	uint16_t RX3 = REMOTE_LOWER;
//
//
//	float magic_number = 6.0f;	// the tracking gain is too large
//	float dt = COLSAMPLE * SAMPLE_TIME;
//	float K[3] = {-284.7354, -58.2616, 35.8509};     // 2022.7.25 changed to IEEE thesis version
//	float tk_bound = 0.2f;	//omega_bounded
//	float tk_diff_bound = 0.1f;
//
//	int delta_refu16_tmp = 0;
//	float delta_ref = MXHEX_CENTER* PI / 180.0f;
//	uint8_t position_state;
//	delta_refu16_tmp = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
//	position_state = dxl_get_result();
//	if (position_state == COMM_RXSUCCESS) {
//		if (delta_refu16_tmp < MXUPPER_BOUND && delta_refu16_tmp > MXLOWER_BOUND) {
//			delta_ref = (float) (MXHEX_CENTER - delta_refu16_tmp) * MXHEX2DEC * DEG2RAD;
//		}
//	}
//
//	float v0 = vel;
//
//	tracking_omega = tracking_omega/magic_number; //need to change bound
//	static float tracking_omega_old = 0.0f;
//	if(fabsf(tracking_omega - tracking_omega_old) > tk_diff_bound){		// ignore if omega command differential is large in original
//		if(tracking_omega > 0.0f) tracking_omega = tracking_omega_old + tk_diff_bound;
//		else tracking_omega = tracking_omega_old - tk_diff_bound;
//	}
//	tracking_omega_old = tracking_omega;	// constrain the omega differential
//
//	float tracking_delta = atanf(b * (tracking_omega) / v0) / sinf(epsilon);
//	if (fabsf(tracking_delta) > tk_bound) {
//		tracking_delta = tracking_delta > 0 ? tk_bound : -tk_bound;
//	}
//
//	start_read(&RX1, &RX2, &RX3);
//	uint16_t top = 1861;
//	uint16_t dow = 1179;
//	RX1 = RX1 > REMOTE_CENTER * 1.1 ? top : RX1;
//	RX1 = RX1 < REMOTE_CENTER * 0.9 ? dow : RX1;
//	float remote_gain  = -12.0f;
//	float delta_remote = remote_gain * PI / 180.0f * (RX1 - REMOTE_CENTER) / (REMOTE_UPPER - REMOTE_CENTER);
////	float delta_remote = (REMOTE_CENTER - RX1) / 2 * PI / 180.0f; // method used in PD control, not sure which is better?
//	static float delta_remote_old = 0.0f;
//	delta_remote = LowpassFilter(delta_remote,delta_remote_old,REMOTE_LOWPASS_FREQUENCY,dt);
//	delta_remote_old = delta_remote;
//
//#if TESTEKF == 1
//	tracking_delta = 0.0f;
//#endif
//
//
//
//	float deltaD = delta_remote + tracking_delta;
//	*remote_angle = deltaD;
//	deltaD = -deltaD; // change signs so steering ccw is positive?
//
//	float x[3] = { theta + THETA_COM * PI / 180.0f, theta_dot, delta_ref};
//	float xd[3] = { v0 * v0 * sinf(epsilon) / (-GRAVITY * b) * deltaD, 0, deltaD };
//	for (int i = 0; i < 3; i++) {
//		x[i] = x[i] + xd[i];  // 2022.7.25 changed to "-"
//	}
//	float input_u = 0;
//	for (int i = 0; i < 3; ++i) {
//		input_u = input_u + K[i] * x[i];
//	}
////	printf("= %4.3f %4.3f %4.3f\r\n", input_u, deltaD * (v0 / a),delta_remote);
//	input_u = input_u + deltaD * (v0 / a);
//
//	float frequency = v0 / a;
//	if (v0 < V_STD) {
//		vo_flag = 0;
//	}
//
//	if (vo_flag) {
//		output = LowpassFilter(input_u, output_old, frequency, dt);
//		output = output / frequency;
//		float notch = notch_filter(notch_old,notch_old_old,output,output_old,output_old_old);
//		output_old_old = output_old;
//		output_old = output;
//
//		notch_old_old = notch_old;
//		notch_old = notch;
//	}
//
//	*delta_output = output;
//	printf("= %4.3f %4.3f %4.3f\r\n", output, input_u ,delta_remote);
//#endif
//#if PD == 1
//	float Kp = 5.6;
//	float Kd = 0.09;
//	float theta_y = theta;	//because IMU position change so the variable is y
//	float theta_ydot = theta_dot;
//
//	float delta_control = 0;
//	delta_control = (theta_y * (180 / PI) + THETA_COM) * Kp + theta_ydot * (180 / PI) * Kd; //unit degree
//	output = -delta_control*PI/180.0f;
//
//#endif
//	hexgoal = (uint16_t) ((MXDEC_CENTER - output * 180.0f / PI) / MXHEX2DEC);
//
//#ifdef INIT_MX
//	if ((hexgoal < MXUPPER_BOUND && hexgoal > MXLOWER_BOUND) && vo_flag) {
//		dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hexgoal);
//	}
//#endif
//
//}
//
//float DeltaCmdControl(float delta_output, float delta_cmd) {
//	float p_gain = 0.01f;
//	float i_gain = 0.0f;
//
//	float error = delta_cmd - delta_output;
//	static float integral_error = 0.0f;
//	integral_error = integral_error + error * COLSAMPLE * SAMPLE_TIME;
//
//	return (error * p_gain + i_gain * integral_error);
//}

float notch_filter(float last_notch_delta, float last_last_notch_delta, float output_delta, float last_output_delta, float last_last_output_delta){
	float output = 0;
	output = 1.994877*last_notch_delta-0.998755*last_last_notch_delta+output_delta-1.996*last_output_delta+0.999875*last_last_output_delta;
//	output = 1.99488*last_notch_delta-0.99875*last_last_notch_delta+output_delta-1.9955*last_output_delta+0.99938*last_last_output_delta;
	// s^2+0.01246s+38.81/s^2+0.1246s+38.81 matlab c2d method = 'zoh'
	return output;
}

//=============================================================================================================================================================
//#include "controller.h"
//#include "MxMotor.h"
//#include "math.h"
//#include "can.h"
//
//// Bicycle parameters
//float a = 0.395;      // length of COG to rear wheel
//float b = 1.053;      // length of front wheel to rear wheel (L)
//
//void LMI(float theta, float theta_dot, float tracking_omega, float *remote_angle, float *delta_output, float vel) {
///* @Input: theta (bicycle roll angle), theta_dot, omega_command(rad), delta_output(rad), velocity from state(mu[3])
// * @note:  When #define EKF, the predicted velocity is fed into the "vel" argument
// * @note:  When EKF "not" defined, the current velocity (from encoder) is fed into "vel"
// **/
//
//    // Parameter initialization
//    uint16_t hexgoal       = MXHEX_CENTER;                 // center of MX-motor
//    uint8_t  vo_flag       = 1;                            // flag of whether current state velocity > V_STD(1.49)
//    float    output        = 0.0f;                         // output command
//    float    magic_number  = 6.0f;	                       // the tracking gain is too large
//	float    dt            = COLSAMPLE * SAMPLE_TIME;      // sample time
//	float    tk_bound      = 0.2f;	                       // bound tracking steer angle
//	float    tk_diff_bound = 0.1f;                         // bound tracking omega
//    float    v0            = vel;
//    uint16_t remote_gain   = 2;		                       // is is bigger the remote control is smaller
//
//    static float output_old = 0.0f;
//	static float output_old_old = 0.0f;
//	static float notch_old = 0.0f;
//	static float notch_old_old = 0.0f;
//
//	// Balance control gain (Solve LMI for balance)
//    float    K[3] = {-266.1011,  -43.6233  , 17.3143};
//
//    // Remote control center value
//	uint16_t RX1 = REMOTE_CENTER;
//	uint16_t RX2 = REMOTE_CENTER;
//	uint16_t RX3 = REMOTE_LOWER;
//	static uint16_t RX1_last = REMOTE_CENTER;
//
//    // Enable remote control
//#ifdef REMOTE_CONTROL
//	start_read(&RX1, &RX2, &RX3);
//	uint16_t top = 1861;
//	uint16_t dow = 1179;
//	RX1 = RX1 > REMOTE_CENTER * 1.1 ? top : RX1;
//	RX1 = RX1 < REMOTE_CENTER * 0.9 ? dow : RX1;
//#endif
//
//    // Control delta angle according to the controller wheel(RX1)
//#ifdef REMOTE_CONTROL
//	if (RX1 < REMOTE_LOWER * (1 - REMOTE_ERROR)|| RX1 > REMOTE_UPPER * (1 + REMOTE_ERROR)) { // afford 10% error, to deal with no remote data
//		RX1 = REMOTE_CENTER;	                                                             // meaning no control
//	} else {
//		RX1 = LowpassFilter(RX1, RX1_last, REMOTE_LOWPASS_FREQUENCY, dt);                    // Defined in PD control
//		RX1_last = RX1;
//	}
//#endif
//
//    // Initialize MX-motor
//    float delta_ref = MXDEC_CENTER;
//    delta_ref = ReadDeltaAngle();
//
//    // Constrain change in tracking omega
//    tracking_omega = tracking_omega/magic_number;                       // Scale down by some magic number
//	static float tracking_omega_old = 0.0f;
//	if(fabsf(tracking_omega - tracking_omega_old) > tk_diff_bound){
//		if(tracking_omega > 0.0f){
//            tracking_omega = tracking_omega_old + tk_diff_bound;
//        } else{
//            tracking_omega = tracking_omega_old - tk_diff_bound;
//        }
//	}
//	tracking_omega_old = tracking_omega;
//
//    // Transforming the omega_d command from tracking to cornering delta_d (steering angle)
//	float tracking_delta = atanf(b * (tracking_omega) / v0) / sinf(epsilon);  // (unit: rad?)
//
//    // Clamp the steering angle
//	if (fabsf(tracking_delta) > tk_bound){
//        if(tracking_delta > 0){
//            tracking_delta = tk_bound;
//        } else{
//            tracking_delta = -tk_bound;
//        }
//	}
//
//    // Turn off tracking if unwanted
//#if TESTEKF == 1
//	tracking_delta = 0.0f;
//#endif
//
//    // Steering comand = remote control command + tracking control command (all ahieved with LMI controller)
//    float deltaD = (REMOTE_CENTER - RX1)/remote_gain*MXHEX2DEC*DEG2RAD + tracking_delta;
//	*remote_angle = deltaD;
//
//    // deltaD = -deltaD;
//
//    // Build States(x) for feedback (Using u = Kx, not u = -Kx), considering "Cornering"
//    float x[3]  = { theta+THETA_COM*PI/180.0f             , theta_dot, delta_ref };  // [theta, theta_dot, delta]
//	float xd[3] = { v0*v0*sinf(epsilon)/(GRAVITY*b)*deltaD,         0,    deltaD };  // Don't consider acceleration; theta_d = v0^2sin(eps)*delta/gb
//
//    // Feedback control with "Cornering"
//    for (int i = 0; i < 3; i++) {
//		x[i] = x[i] - xd[i];
//	}
//	float input_u = 0;
//	for (int i = 0; i < 3; ++i) {
//		input_u = input_u + K[i] * x[i];
//	}
//
//    // feedback u = K*x_bar + u_d, for details view MATLAB simulation.slx
//	input_u = input_u + deltaD * (v0 / a);
//
//	float frequency = v0 / a;
//
//	// Raise flag when speed acheives "controllable speed (1.49m/s)"
//	if (v0 < V_STD) {
//		vo_flag = 0;
//	}
//
//	// When flag raised, start control (this is not the case in PD control, maybe consider removal)
//	if (vo_flag) {
//		output = LowpassFilter(input_u, output_old, frequency, dt);
//		output = output / frequency;
//		int aa = output * 180.0f / PI*1000;
//		float notch = notch_filter(notch_old,notch_old_old,output,output_old,output_old_old);
//		output_old_old = output_old;
//		output_old = output;
//
//		notch_old_old = notch_old;
//		notch_old = notch;
//
//		// output = notch;
//    }
//
//	*delta_output = output;  // should be in radians
//
//	// Write to MX-motor
//	hexgoal = (uint16_t) ((MXDEC_CENTER - output * 180.0f / PI) / MXHEX2DEC);
//#ifdef INIT_MX
//	if ((hexgoal < MXUPPER_BOUND && hexgoal > MXLOWER_BOUND) && vo_flag) {
//		dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, hexgoal);
//	}
//#endif
//
//}
//
//float notch_filter(float last_notch_delta, float last_last_notch_delta, float output_delta, float last_output_delta, float last_last_output_delta){
//	float output = 0;
//	output = 1.994877*last_notch_delta-0.998755*last_last_notch_delta+output_delta-1.996*last_output_delta+0.999875*last_last_output_delta;
////	output = 1.99488*last_notch_delta-0.99875*last_last_notch_delta+output_delta-1.9955*last_output_delta+0.99938*last_last_output_delta;
//	// s^2+0.01246s+38.81/s^2+0.1246s+38.81 matlab c2d method = 'zoh'
//	return output;
//}
