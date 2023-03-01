/*
 * EKF.c
 *
 *  Created on: 2020�~7��7��
 *      Author: ldsc8
 */

#include "EKF.h"
#include "MatrixCalculate.h"
#include "usart.h"
#include "math.h"
#include "QuadrantAngle.h"

extern uint8_t gpsdata[36];  // for details, read stm32f4xx_it.c usart3
extern uint8_t update_gps;

void readGPS(double  *latitude, double  *longitude){	//latitude 23.5 N  longitude 120 E  reference UBX-NAV-POSLLH
	uint32_t lat = gpsdata[17] << 24U | gpsdata[16] << 16U | gpsdata[15] << 8U | gpsdata[14];
	uint32_t lon = gpsdata[13] << 24U | gpsdata[12] << 16U | gpsdata[11] << 8U | gpsdata[10];

	*latitude = lat*1e-7;
	*longitude = lon*1e-7;
}

void ReadAccuracyEstimate(float *horizontal, float *vertical){
	uint32_t ver = gpsdata[33] << 24U | gpsdata[32] << 16U | gpsdata[31] << 8U | gpsdata[30];
	uint32_t hor = gpsdata[29] << 24U | gpsdata[28] << 16U | gpsdata[27] << 8U | gpsdata[26];

	*horizontal = hor*1e-3;
	*vertical = ver*1e-3;
}

double distance_gps(double latitude_first, double longitude_first, double latitude_cutrrent, double longitude_current){
	//reference
	//http://www.movable-type.co.uk/scripts/latlong.html
	const double R = 6371000; // earth radius 6371km
	double p1 = latitude_cutrrent * DEG2RAD_D;
	double p2 = latitude_first * DEG2RAD_D;
	double dp = (latitude_first - latitude_cutrrent) * DEG2RAD_D;
	double dl = (longitude_first - longitude_current) * DEG2RAD_D;

	double x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
	double y = 2 * atan2(sqrt(x), sqrt(1-x));

	return R * y; //unit meter
}

void gpsXY(double lat, double lon, float *xy){
	float x_axis_range = 250;		// where I think the map is on -250~250 square
	float y_axis_range = 250;		//
	double gps_driftx = 249472.0;	// to adjust gps x,y coordinate to near the original point
	double gps_drifty = 2743027.0;
// reference
	//http://sask989.blogspot.com/2012/05/wgs84totwd97.html
    double a = 6378137.0;
    double b = 6356752.3142451;
    double lon0 = 121 * DEG2RAD_D;
    double k0 = 0.9999;
    double dx = 250000;
    double dy = 0;
	lat = lat*DEG2RAD_D;
	lon = lon*DEG2RAD_D;

    double e = (1 - pow(b,2) / pow(a,2));
    double e2 = (1 - pow(b, 2) / pow(a, 2)) / (pow(b, 2) / pow(a, 2));

	double V = a / sqrt(1 - e * pow(sin(lat), 2));
	double T = pow(tan(lat), 2);
	double C = e2 * pow(cos(lat), 2);
	double A = cos(lat) * (lon - lon0);
	double M = a
			* ((1.0 - e / 4.0 - 3.0 * pow(e, 2) / 64.0
					- 5.0 * pow(e, 3) / 256.0) * lat
					- (3.0 * e / 8.0 + 3.0 * pow(e, 2) / 32.0
							+ 45.0 * pow(e, 3) / 1024.0)
							* sin(2.0 * lat)
					+ (15.0 * pow(e, 2) / 256.0
							+ 45.0 * pow(e, 3) / 1024.0)
							* sin(4.0 * lat)
					- (35.0 * pow(e, 3) / 3072.0) * sin(6.0 * lat));
    // x
    double x = dx + k0 * V * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2) + 72 * C - 58 * e2) * pow(A, 5) / 120);
   // y
    double y = dy + k0 * (M + V * tan(lat) * (pow(A, 2) / 2 + (5 - T + 9 * C + 4 * pow(C, 2)) * pow(A, 4) / 24 + ( 61 - 58 * T + pow(T, 2) + 600 * C - 330 * e2) * pow(A, 6) / 720));

    float tmp_xy[2];
    tmp_xy[0] = (float)(x-gps_driftx);
    tmp_xy[1] = (float)(y-gps_drifty);
    if(fabs(tmp_xy[0]) < x_axis_range && fabs(tmp_xy[1]) < y_axis_range){
        xy[0] = (float)(x-gps_driftx);
        xy[1] = (float)(y-gps_drifty);
    }
}



void EKF_filter(float *mu,float theta_x,float *acc,float *gyro,float *point_xy,float vel_cur, float theta_y, float horizontal_accuracy){
/* @input:    mu: state[X, Y, phi, v0], theta_x: roll angle, acc: accelerometer data,
 * @input:    gyro: gyroscope data, point_xy: GPS position, vel_curr: encoder data
 * @input:    theta_y: pitch angle, horizontal_accuracy: GPS data
 * @function: Do EKF sensor data fusion with the according input data.
 * */

//	////////////////////////////////////// Parameter EKF
//
//	float Q[9] = {			//sensor covariance 1.GPS X, 2.GPS Y, 3.Encoder
//			1.2f, 0.0, 0.0,
//			0.0, 1.2f, 0.0,
//			0.0, 0.0, 0.03f
//	};

	float Q[9] = {			//sensor covariance 1.GPS X, 2.GPS Y, 3.Encoder
			1.5f, 0.0, 0.0,
			0.0, 1.5f, 0.0,
			0.0, 0.0, 0.1f
	};
	float gps_dt = 0.2f;
	Q[0] = horizontal_accuracy * horizontal_accuracy *2.198*10.0f / gps_dt + 0.5f;	// except 30cm variance (0.5f)
	// CEP to standard deviation: CEP(0.5) = 1.1774*stdev(sigma)
	Q[4] = horizontal_accuracy * horizontal_accuracy *2.198*10.0f / gps_dt + 0.5f;	// CEP to standard deviation 50% to one sigma : 0.6745*sigma = 50%


//	////////////////////////////////////// Parameter EKF
	float theta = theta_x+THETA_COM*(PI/180.0f);
	float acc_x = acc[0];

	acc_x = acc_x + GRAVITY*sinf(theta_y);
	float gyro_z = gyro[2];
	float vel = vel_cur;
	float p_X = point_xy[0];
	float p_Y = point_xy[1];

	float position_omega = gyro_z/cosf(theta);

	float X   = mu[0];
	float Y   = mu[1];
	float phi = mu[2];
	float v0  = mu[3];

	float G13 = -v0*EKF_TIME*sinf(phi);
	float G14 = EKF_TIME*cosf(phi);
	float G23 = v0*EKF_TIME*cosf(phi);
	float G24 = EKF_TIME*sinf(phi);

	float Gt[16] = {1,0,G13,G14,
					0,1,G23,G24,
					0,0,  1,  0,
					0,0,  0,  1};

	float V11 = EKF_TIME*EKF_TIME*cosf(phi);
	float V12 = -v0*EKF_TIME*EKF_TIME*sinf(phi);
	float V21 = EKF_TIME*EKF_TIME*sinf(phi);
	float V22 = v0*EKF_TIME*EKF_TIME*cosf(phi);

	float Vt[8] = {V11,      V12,
			       V21,      V22,
			       0,   EKF_TIME,
			       1,          0};

//	float M11 = alpha1*powf(acc_x,2) + alpha2*powf(position_omega,2);
//	float M22 = alpha3*powf(acc_x,2) + alpha4*powf(position_omega,2);
//	float aa = 33.3;
//	printf("MM %.10f %.10f %.3f\r\n", M11, M22, aa);
//
//	float Mt[4] = {M11,    0,
//			       0,    M22};
	float Mt[4] = {0.0001,    0,
				   0,    0.0001};

	mu[0] = X   + v0*EKF_TIME*cosf(phi);
	mu[1] = Y   + v0*EKF_TIME*sinf(phi);
	mu[2] = phi + position_omega*EKF_TIME;
	mu[3] = v0  + acc_x*EKF_TIME;

	float GtCov[16];
	float GtT[16];
	float GtCovGtT[16];
	float VtMt[8];
	float VtT[8];
	float VtMtVtT[16];

static float covariance[16] = {
	0.0f,0.0f,0.0f,0.0f,
	0.0f,0.0f,0.0f,0.0f,
	0.0f,0.0f,0.2f,0.0f,
	0.0f,0.0f,0.0f,0.0f
};

	mattimes(Gt, 4, 4, covariance, 4, 4, GtCov);
	transpose(Gt, 4, 4, GtT);
	mattimes(GtCov, 4, 4, GtT, 4, 4,GtCovGtT);

	mattimes(Vt, 4, 2, Mt, 2, 2, VtMt);				//up-down:4 left-right:2
	transpose(Vt, 4, 2, VtT);
	mattimes(VtMt, 4, 2, VtT, 2, 4, VtMtVtT);

	matplus(GtCovGtT, 4, 4, VtMtVtT, covariance);

	if(update_gps == 1){ ///////////////////////////////////////////////////////////////////////////////////// if update

		float z_hat[3];
		z_hat[0] = mu[0];
		z_hat[1] = mu[1];
		z_hat[2] = mu[3];

		float H[12] = {
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 0, 1
		};

		float HCov[12];
		float HT[12];
		float HCovHT[9];
		mattimes(H, 3, 4, covariance, 4, 4, HCov);
		transpose(H, 3, 4, HT);
		mattimes(HCov, 3, 4, HT, 4, 3, HCovHT);

		float S[9];
		matplus(HCovHT, 3, 3, Q, S);

		float CovHT[12];
		float inv_S[9];
		float K[12];
		mattimes(covariance, 4, 4, HT, 4, 3, CovHT);
		inv3(S,inv_S);
		mattimes(CovHT, 4, 3, inv_S, 3, 3, K);

		float z_update[3] = {p_X, p_Y, vel};

		float z[3];
		z[0] = z_update[0] - z_hat[0];
		z[1] = z_update[1] - z_hat[1];
		z[2] = z_update[2] - z_hat[2];

		float Kz[4];
		mattimes(K, 4, 3, z, 3, 1, Kz);
		for(int i=0; i<4; i++){
			mu[i] = mu[i]+Kz[i];
		}

		float KH[16];
		float I_KH[16];
		float cov_tmp[16];
		float I_4[16]={
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,1
		};
		mattimes(K, 4, 3, H, 3, 4, KH);
		for(int i=0;i<16;i++){
			KH[i] = -KH[i];
		}
		matplus(I_4, 4, 4, KH, I_KH);
		mattimes(I_KH, 4, 4, covariance, 4, 4, cov_tmp);	//it need tmp because covariance will be
		for(int i=0; i<16; i++){
			covariance[i] = cov_tmp[i];
		}
		update_gps = 0;
	}

	Qangle(&mu[2]);
}






