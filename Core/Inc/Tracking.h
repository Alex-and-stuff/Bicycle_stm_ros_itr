/*
 * Tracking.h
 *
 *  Created on: 2021¦~3¤ë18¤é
 *      Author: ldsc8
 */

#ifndef INC_TRACKING_H_
#define INC_TRACKING_H_

// generate waypoint trajectory line
void WayPointLine(float *vd,float *wd, float *stateD, float *BeginPoint, float *EndPoint, float *Vrange, uint32_t clock_time);
// generate waypoint trajectory arc
void WayPointCircle(float *vd,float *wd, float *stateD, float *BeginPoint, float *EndPoint, float *Vrange, float *CirclePoint, float Radius, uint32_t clock_time,uint8_t line);
// solve the point two circle
void CroTwoCir(float *center, float r, float *circle, float R, float *P1, float *P2);
float DistanceD(float *P1, float *P2);
// tracking function
void Tracking(float *mu, float *BeginPoint, float *EndPoint, float *CirclePoint, float radius, uint8_t turn_flag, uint8_t* line, uint32_t *time_shift, float *control, float *stateD);
// the map you need setup waypoint if you want to change the map
void Mapping(float *way_point_begin, float *way_point_end, float *circle_point,float *radius, uint8_t *turn_flag, uint8_t line);



#endif /* INC_TRACKING_H_ */
