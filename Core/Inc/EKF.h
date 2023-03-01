/*
 * EKF.h
 *
 *  Created on: 2020¦~7¤ë7¤é
 *      Author: ldsc8
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

void readGPS(double *latitude, double *longitude);
void ReadAccuracyEstimate(float *horizontal, float *vertical);
double distance_gps(double latitude_first, double longitude_first, double latitude_cutrrent, double longitude_current);
void EKF_filter(float *mu,float theta_x,float *acc,float *gyro,float *point_xy,float vel_cur, float theta_y, float horizontal_accuracy);
void gpsXY(double lat, double lon, float *xy);

#define DEG2RAD_D 0.0174532925199432957




#endif /* INC_EKF_H_ */
