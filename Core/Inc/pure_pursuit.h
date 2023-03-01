/*
 * pure_pursuit.h
 *
 *  Created on: 2020¦~7¤ë14¤é
 *      Author: ldsc8
 */

#ifndef INC_PURE_PURSUIT_H_
#define INC_PURE_PURSUIT_H_

#include <main.h>
#include <math.h>
#include <stdlib.h>

float pur_pursuit(float *way_point_begin, float *way_point_end,float *mu, float LKAD_distance,float *cross_point);
float pur_pursuit_cir(float *circle_p, float radius,float *mu, float LKAD_distance,  float *cross_point);

#endif /* INC_PURE_PURSUIT_H_ */
