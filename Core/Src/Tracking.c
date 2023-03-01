/*
 * Tracking.c
 *
 *  Created on: 2021�~3��18��
 *      Author: ldsc8
 */
#include <main.h>
#include <math.h>
#include "Tracking.h"
#include "QuadrantAngle.h"
#include "MatrixCalculate.h"
#include <stdio.h>

#define GPSRTK 0

void Tracking(float *mu, float *begin_point, float *end_point, float *circle_point,
		float radius, uint8_t turn_flag, uint8_t* line, uint32_t *time_shift,
		float *control,float *stateD) {

//	////////////////////////////////////// Parameter Tracking
//
	float change_time = 0.1f;		// if (Xd,Yd) and end_point distance less than 10 centimeter , line will change stage
	float velocity_range_in_line[2] = { 2.0f, 2.0f };
	float velocity_range_in_circle[2] = { 2.0f, 2.0f };
//
//	////////////////////////////////////// Parameter Tracking

	float vd = 0.0f;
	float wd = 0.0f;


	uint32_t clock_time = HAL_GetTick();
	clock_time = clock_time - *time_shift;
	if (turn_flag == 0) {

		WayPointLine(&vd, &wd, stateD, begin_point, end_point, velocity_range_in_line, clock_time);
	} else {

		WayPointCircle(&vd, &wd, stateD, begin_point, end_point, velocity_range_in_circle, circle_point, radius, clock_time, *line);
	}

	float Rotate2Bike[9] = { 0.0f };
	Rotate2Bike[0] = cosf(mu[2]);
	Rotate2Bike[1] = sinf(mu[2]);
	Rotate2Bike[3] = -sinf(mu[2]);
	Rotate2Bike[4] = cosf(mu[2]);
	Rotate2Bike[8] = 1.0f;

	float stateErr[3] = { 0.0f };
	stateErr[0] = stateD[0] - mu[0];
	stateErr[1] = stateD[1] - mu[1];
	stateErr[2] = stateD[2] - mu[2];
	Qangle(&stateErr[2]);
	float bike_e[3];
	mattimes(Rotate2Bike, 3, 3, stateErr, 3, 1, bike_e);

	float K[6] = { 0.0f }; // K = [-0.5 0 0 ; 0 -1.5798 -9.7577]
	K[0] = -1.5737f;		// -0.5	-0.6253
	K[4] = -4.6026f;	// -1.6469	-1.4976
	K[5] = -4.9070f;	// -1.2876	-2.0598

	float u12[2];
	mattimes(K, 2, 3, bike_e, 3, 1, u12);

	control[0] = vd * cosf(bike_e[2]) - u12[0];
	control[1] = wd - u12[1];

	if (DistanceD(stateD, end_point) < change_time) {
		*line = *line + 1;
		*time_shift = *time_shift + clock_time;
	}
}

void WayPointLine(float *vd, float *wd, float *stateD, float *begin_point, float *end_point, float *Vrange, uint32_t clock_time) {

	float tracking_time = clock_time / 1000.0f;
	float vi = Vrange[0];	// initial velocity
	float vf = Vrange[1];	// final velocity
	float dis_total = DistanceD(begin_point, end_point);	// line distance
	float del_t = 2 * dis_total / (vi + vf);			// total time
	float acc = (vf - vi) / del_t;						// average acceleration

	float dis_now = (vi * tracking_time + 0.5 * acc * tracking_time * tracking_time) / dis_total;// x=v0t+(1/2)at^2
	float Xd = (end_point[0] - begin_point[0]) * dis_now + begin_point[0];
	float Yd = (end_point[1] - begin_point[1]) * dis_now + begin_point[1];
	float Thetad = atan2f(end_point[1] - begin_point[1], end_point[0] - begin_point[0]);

	*vd = (vf - vi) * dis_now + vi;
	*wd = 0.0f;
	stateD[0] = Xd;
	stateD[1] = Yd;
	stateD[2] = Thetad;
}

void WayPointCircle(float *vd, float *wd, float *stateD, float *begin_point,
		float *end_point, float *Vrange, float *circle_point, float radius,
		uint32_t clock_time, uint8_t line) {

	float tracking_time = clock_time / 1000.0f;
	float vi = Vrange[0];	// initial velocity
	float vf = Vrange[1];	// final velocity
	float dis_two = DistanceD(begin_point, end_point);
	float gamma = acosf((2 * powf(radius, 2) - powf(dis_two, 2)) / (2 * powf(radius, 2)));
	float length_total = radius * gamma;
	float del_t = 2 * length_total / (vi + vf);
	float acc = (vf - vi) / del_t;
	float length_now = vi * tracking_time + 0.5 * acc * tracking_time * tracking_time;
	float theta_now = length_now / radius;
	float dis_now = sqrt(2 * powf(radius, 2) - 2 * powf(radius, 2) * cosf(theta_now));
	float P1[2];
	float P2[2];
	float cross[2];
	CroTwoCir(circle_point, radius, begin_point, dis_now, P1, P2);

	float tmp1 = DistanceD(P1, end_point);
	float tmp2 = DistanceD(P2, end_point);
	if (tmp1 <= tmp2) {
		cross[0] = P1[0];
		cross[1] = P1[1];
	} else {
		cross[0] = P2[0];
		cross[1] = P2[1];
	}
	float vectorCenCro[2];
	vectorCenCro[0] = circle_point[0] - cross[0];
	vectorCenCro[1] = circle_point[1] - cross[1];

	float tan_tmp[2] = { 1, 0 };
	tan_tmp[1] = -vectorCenCro[0] / vectorCenCro[1];

	float check_head = vectorCenCro[0] * tan_tmp[1] - vectorCenCro[1] * tan_tmp[0];
	float Thetad = 0;
	if (check_head > 0) {
		Thetad = atan2f(tan_tmp[1], tan_tmp[0]);
	} else {
		Thetad = atan2f(-tan_tmp[1], -tan_tmp[0]);
	}

	*vd = (vf - vi) * (tracking_time / del_t) + vi;
	float total_time = length_total*2/(vi+vf);
	float avg_velocity = (vi+vf)/2.0f;
	float avg_omega = avg_velocity/radius;
	float omega_tmp = 0;
	float turn_omega_gain = 0.0f;	// need to change because of magic number
	switch (line) {
	case 2: {
		turn_omega_gain = 6.5f;		//6.0
	}
		break;
	case 4: {
		turn_omega_gain = 16.0f;	//14.0
	}
		break;
	case 6: {
		turn_omega_gain = 11.0f;	//9.0
	}
		break;
	case 8: {
		turn_omega_gain = 13.5f;	//12.0
	}
		break;
	default:
		turn_omega_gain = 10.0f;
		break;
	}

	if(tracking_time < total_time/4.0f){
		omega_tmp = turn_omega_gain*avg_omega*(tracking_time/(total_time/4.0f));

	}else if(tracking_time > total_time*3.0f/4.0f){
		omega_tmp = turn_omega_gain*avg_omega-turn_omega_gain*avg_omega*(tracking_time-total_time*3.0f/4.0f)/(total_time/4.0f);
	}else{
		omega_tmp = turn_omega_gain*avg_omega;
	}
//	omega_tmp = turn_omega_gain*avg_omega;

	*wd = -omega_tmp; //omega_tmp
	stateD[0] = cross[0];
	stateD[1] = cross[1];
	stateD[2] = Thetad;
}

float DistanceD(float *P1, float *P2) {
	float dis_X = powf(P1[0] - P2[0], 2);
	float dis_Y = powf(P1[1] - P2[1], 2);
	float ans = sqrtf(dis_X + dis_Y);
	return ans;
}

void CroTwoCir(float *center, float r, float *circle, float R, float *P1, float *P2) {	//cross point two circle
	float px = circle[0];
	float py = circle[1];
	float R2 = R * R;

	float c1 = center[0];
	float d1 = center[1];
	float r2 = r * r;

	float point_line = sqrt(powf((c1 - px), 2) + powf((d1 - py), 2));	//center distance					//reference   https://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect

	float tmp2 = (R2 - r2) / (2 * powf(point_line, 2));
	float tmp3 = 0.5 * sqrt(2 * (R2 + r2) / powf(point_line, 2) - powf(R2 - r2, 2) / powf(point_line, 4) - 1);
	if (R == 0) {
		P1[0] = px;
		P1[1] = py;
		P2[0] = px;
		P2[1] = py;
	} else {
		P1[0] = 0.5f * (px + c1) + tmp2 * (c1 - px) + tmp3 * (d1 - py);
		P1[1] = 0.5f * (py + d1) + tmp2 * (d1 - py) + tmp3 * (px - c1);
		P2[0] = 0.5f * (px + c1) + tmp2 * (c1 - px) - tmp3 * (d1 - py);
		P2[1] = 0.5f * (py + d1) + tmp2 * (d1 - py) - tmp3 * (px - c1);
	}
}

void Mapping(float *way_point_begin, float *way_point_end, float *circle_point,
		float *radius, uint8_t *turn_flag, uint8_t line) {
#if GPSRTK == 0
	float map_be1[2] = { 150.0743f, 129.9763f };
	float map_ed1[2] = { 151.5227f, 120.2719f };
	float map_be2[2] = { 70.7294f, 11.5641f };
	float map_ed2[2] = { 60.2217f, 8.9381f };
	float map_be3[2] = { 10.06f, 32.6295f };
	float map_ed3[2] = { 7.3027f, 43.6874f };
	float map_be4[2] = { 95.0358f, 161.32f };
	float map_ed4[2] = { 104.8836f, 162.8157f };
	float circle_p1[2] = { 146.031f, 124.4127f };
	float circle_p2[2] = { 63.8724f, 16.6657f };
	float circle_p3[2] = { 13.2039f, 39.2861f };
	float circle_p4[2] = { 100.717f, 157.0824f };
	float radius1 = 6.8777f;
	float radius2 = 8.5466f;
	float radius3 = 7.3617f;
	float radius4 = 7.0875f;

	switch (line) {
	case 1: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_ed1[i];
			way_point_end[i] = map_be2[i];
		}
		*turn_flag = 0;
		//		  				mu[2] = -2.2311;
	}
		break;
	case 2: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_be2[i];
			way_point_end[i] = map_ed2[i];

			circle_point[i] = circle_p2[i];
		}
		*radius = radius2;
		*turn_flag = 1;
	}
		break;
	case 3: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_ed2[i];
			way_point_end[i] = map_be3[i];
		}
		*turn_flag = 0;
		//		  				mu[2] = 2.6925;
	}
		break;
	case 4: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_be3[i];
			way_point_end[i] = map_ed3[i];

			circle_point[i] = circle_p3[i];
		}
		*radius = radius3;
		*turn_flag = 1;
	}
		break;
	case 5: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_ed3[i];
			way_point_end[i] = map_be4[i];
		}
		*turn_flag = 0;
		//		  				mu[2] = 0.93275;
	}
		break;
	case 6: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_be4[i];
			way_point_end[i] = map_ed4[i];

			circle_point[i] = circle_p4[i];
		}
		*radius = radius4;
		*turn_flag = 1;
	}
		break;
	case 7: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_ed4[i];
			way_point_end[i] = map_be1[i];
		}
		*turn_flag = 0;
		//		  				mu[2] = -0.6333;
	}
		break;
	case 8: {
		for (int i = 0; i < 2; i++) {
			way_point_begin[i] = map_be1[i];
			way_point_end[i] = map_ed1[i];

			circle_point[i] = circle_p1[i];
		}
		*radius = radius1;
		*turn_flag = 1;
	}
		break;
	default:
		printf("map error\r\n");	// maybe need add ERROR function
		break;
	}

#endif

#if GPSRTK ==1
	float map_be1[2] = {150.0743f, 129.9763f};
	float map_ed1[2] = {151.5227f, 120.2719f};
	float map_be2[2] = {69.8614f, 11.9645f};
	float map_ed2[2] = {59.3177f, 9.3647f};
	float map_be3[2] = {9.5084f, 32.8896f};
	float map_ed3[2] = {6.7511f, 43.9476f};
	float map_be4[2] = {94.5524f, 161.6717f};
	float map_ed4[2] = {104.4001f, 163.1678f};
	float circle_p1[2] = {146.0311f, 124.4125f};
	float circle_p2[2] = {62.9919f, 17.1439f};
	float circle_p3[2] = {12.6524f, 39.5463f};
	float circle_p4[2] = {100.2337f, 157.4344f};
	float radius1 = 6.8777f;
	float radius2 = 8.6033f;
	float radius3 = 7.3618f;
	float radius4 = 7.0874f;

	switch (line) {
		case 1: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_ed1[i];
				way_point_end[i] = map_be2[i];
			}
			*turn_flag = 0;
			//		  				mu[2] = -2.2311;
		}
		break;
		case 2: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_be2[i];
				way_point_end[i] = map_ed2[i];

				circle_point[i] = circle_p2[i];
			}
			*radius = radius2;
			*turn_flag = 1;
		}
		break;
		case 3: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_ed2[i];
				way_point_end[i] = map_be3[i];
			}
			*turn_flag = 0;
			//		  				mu[2] = 2.6925;
		}
		break;
		case 4: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_be3[i];
				way_point_end[i] = map_ed3[i];

				circle_point[i] = circle_p3[i];
			}
			*radius = radius3;
			*turn_flag = 1;
		}
		break;
		case 5: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_ed3[i];
				way_point_end[i] = map_be4[i];
			}
			*turn_flag = 0;
			//		  				mu[2] = 0.93275;
		}
		break;
		case 6: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_be4[i];
				way_point_end[i] = map_ed4[i];

				circle_point[i] = circle_p4[i];
			}
			*radius = radius4;
			*turn_flag = 1;
		}
		break;
		case 7: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_ed4[i];
				way_point_end[i] = map_be1[i];
			}
			*turn_flag = 0;
			//		  				mu[2] = -0.6333;
		}
		break;
		case 8: {
			for (int i = 0; i < 2; i++) {
				way_point_begin[i] = map_be1[i];
				way_point_end[i] = map_ed1[i];

				circle_point[i] = circle_p1[i];
			}
			*radius = radius1;
			*turn_flag = 1;
		}
		break;
		default:
		printf("map error\r\n");	// maybe need add ERROR function
		break;
	}
#endif

}

