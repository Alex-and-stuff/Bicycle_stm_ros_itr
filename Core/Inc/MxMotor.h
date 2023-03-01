/*
 * MxMotor.h
 *
 *  Created on: 2019¦~11¤ë27¤é
 *      Author: LDSCB
 */

//#include <stdint.h>
#include "usart.h"
#include "main.h"
#include "dynamixel.h"

#ifndef INC_MXMOTOR_H_
#define INC_MXMOTOR_H_

#define Torque_Enable			24
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_GOAL_SPEED_L			32
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_PRESENT_SPEED_L		38
#define P_PRESENT_SPEED_H		39
#define P_MOVING				46
#define P_GAIN					28
#define I_GAIN					27
#define D_GAIN					26
#define CW_LIMIT				6
#define CCW_LIMIT				8

#define USART_DXL			    0
#define USART_PC			    2

void ClearBuffer256(void);
uint8_t CheckNewArrive(void);
void TxDByte_DXL(uint8_t bTxdData);
uint8_t RxDByte_DXL(void);
void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);
void TxDString(uint8_t *bData);
void TxDWord16(uint16_t wSentData);
void TxDWord16(uint16_t wSentData);
void TxDByte16(uint8_t bSentData);
void TxDByte_PC(uint8_t bTxdData);
void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void mDelay(uint32_t nTime);
void StartDiscount(int32_t StartTime);
uint8_t CheckTimeOut(void);

void Motor(void);
void ttst(void);
void pp(void);
void PIDsetup(int Pgain,int Igain, int Dgain);
void AngleLimit(uint16_t upper_bound, uint16_t lower_bound);
void MotorResponse(uint16_t testangle);							//right positive, left negative
void MxReadSpeed(void);
void Torque_off(void);
void Torque_on(void);
uint8_t TorqueState(void);
float ReadDeltaAngle(void);
float ReadDeltaOmega(void);

#define MOTOR_ID 1





#endif /* INC_MXMOTOR_H_ */


