/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

void setup(void);
void loop(int* val);
void loop2(float* theta, float* theta_dot, float* delta, float*v, float* cmd);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
