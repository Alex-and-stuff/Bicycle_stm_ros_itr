/*
 * QuadrantAngle.c
 *
 *  Created on: 2021¦~3¤ë18¤é
 *      Author: ldsc8
 */
#include "QuadrantAngle.h"
#include "main.h"

void Qangle(float *angle) {
	while ((*angle > PI) || (*angle <= -PI)) {
		if (*angle > 0) {
			*angle = *angle - 2 * PI;
		} else {
			*angle = *angle + 2 * PI;
		}
	}
}

