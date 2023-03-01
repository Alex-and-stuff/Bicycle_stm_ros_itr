///*
// * sensor_fusion.c
//
// *
// *  Created on: 2019¦~11¤ë20¤é
// *      Author: LDSCB
// */
//#include "sensor_fusion.h"
//#include "can.h"
//
////#define DEBUGSENSOR_FUSION 1
/////////////////////////////////////////////////////////////////////////////////////
//	float distance = 1.051; //between wheel axles
//	float alpha = PI;
//
//	float xdot[5] = {0,0,0,0,0};
//	float xMatrix[5] = {-GRAVITY,0,0,0,0};
//	float AMatrix[25] = {0,0,0,0,0,
//						  0,0,0,0,0,
//						  0,0,0,0,0,
//						  0,0,PI,-PI,0,
//						  0,0,1,0,0};
//
//	float BuMatrix[5]  = {0,0,0,0,0};
//	float LCxMatrix[5] = {0,0,0,0,0};
//	float LDvMatrix[5] = {0,0,0,0,0};
//	float LYMatrix[5]  = {0,0,0,0,0};
//	float AxMatrix[5]  = {0,0,0,0,0};
//
//
//	float Observer_Gain[6] = {-1.588, -9.854,
//							  -1.588, -9.886,
//							  -0.493, -3.142};
//
//	float AccelerationS[3] = {-GRAVITY,0,0};
//	float AccelerationC[3]={0, 0, 0};
//	float PhiDot = 0.0;
//	float Theta = 0.0;
//	float Psi = 0.0;	//	volatile
//	float Theta_dot = 0.0;
//
//
//void sensor_fusion(){
//	volatile extern float gyrox;		//omega1
//	volatile extern float gyroy;		//omega2
//	volatile extern float gyroz;		//omega3
//	volatile extern float accex;		//as1
//	volatile extern float accey;		//as2
//	volatile extern float accez;		//as3
////	float V_current = 2.0;
////	if(V_current < V_STD){
////		V_current = V_REF;
////	}
//
//
//	float omega1 = gyroz;	//contrast to  paper
//	float omega2 = gyrox;	//2019/12/31 because IMU put on horizontal so Z to Y¡BY to -X¡BX to Z
//	float omega3 = gyroy;	//
//
//	/////////////////////////////////////// reading sensor
//	xMatrix[4] = V_current;
//
//	AccelerationS[0] = accez*GRAVITY;	// LSM9DS0 unit is GRAVITY
//	AccelerationS[1] = accex*GRAVITY;	// 2019/12/31 because IMU put on horizontal so Z to Y¡BY to -X¡BX to Z
//	AccelerationS[2] = accey*GRAVITY;	//
//
//	AMatrix[1] =  omega3;				//   0  w3 -w2
//	AMatrix[2] = -omega2;				// -w3   0  w1
//	AMatrix[5] = -omega3;				//  w2 -w1   0
//	AMatrix[7] =  omega1;				//
//	AMatrix[10]=  omega2;				//
//	AMatrix[11]= -omega1;				//
//	/////////////////////////////////////// reading sensor
//
//
//	////////////////////////////////////// matrix calculation
//
//	// x_dot = Ax+u+L(Cx+v-y)
//
//	AccelerationC[0] = xMatrix[4]*PhiDot*sin(Theta);	//acx
//	AccelerationC[1] = xMatrix[4]*PhiDot*cos(Theta);	//acy
//	AccelerationC[2] = distance*PhiDot*PhiDot;			//acz
//
//	BuMatrix[3] = -alpha*AccelerationS[2]+alpha*AccelerationC[2];
//	BuMatrix[4] = -AccelerationS[2]+AccelerationC[2];
//
//	LCxMatrix[0] = -0.2*PI*xMatrix[0]-omega3*xMatrix[1]+omega2*xMatrix[2]-omega2*xMatrix[3];
//	LCxMatrix[1] = omega3*xMatrix[0]-0.2*PI*xMatrix[1]-omega1*xMatrix[2]+omega1*xMatrix[3];
//	LCxMatrix[2] = -omega2*xMatrix[0]+omega1*xMatrix[1]+Observer_Gain[0]*xMatrix[2]-Observer_Gain[0]*xMatrix[3]+Observer_Gain[1]*xMatrix[4];
//	LCxMatrix[3] = Observer_Gain[2]*xMatrix[2]-Observer_Gain[2]*xMatrix[3]+Observer_Gain[3]*xMatrix[4];
//	LCxMatrix[4] = Observer_Gain[4]*xMatrix[2]-Observer_Gain[4]*xMatrix[3]+Observer_Gain[5]*xMatrix[4];
//
//	LDvMatrix[0] = -0.2*PI*AccelerationC[0]-omega3*AccelerationC[1]+omega2*AccelerationC[2];
//	LDvMatrix[1] = omega3*AccelerationC[0]-0.2*PI*AccelerationC[1]-omega1*AccelerationC[2];
//	LDvMatrix[2] = -omega2*AccelerationC[0] + omega1*AccelerationC[1]+Observer_Gain[0]*AccelerationC[2];
//	LDvMatrix[3] = Observer_Gain[2]*AccelerationC[2];
//	LDvMatrix[4] = Observer_Gain[4]*AccelerationC[2];
//
//	LYMatrix[0] = -0.2*PI*AccelerationS[0]-omega3*AccelerationS[1]+omega2*AccelerationS[2];
//	LYMatrix[1] = omega3*AccelerationS[0]-0.2*PI*AccelerationS[1]-omega1*AccelerationS[2];
//	LYMatrix[2] = -omega2*AccelerationS[0]+omega1*AccelerationS[1]+Observer_Gain[0]*AccelerationS[2]+Observer_Gain[1]*xMatrix[4];
//	LYMatrix[3] = Observer_Gain[2]*AccelerationS[2]+Observer_Gain[3]*xMatrix[4];
//	LYMatrix[4] = Observer_Gain[4]*AccelerationS[2]+Observer_Gain[5]*xMatrix[4];
//
//	AxMatrix[0] = omega3*xMatrix[1]-omega2*xMatrix[2];
//	AxMatrix[1] = -omega3*xMatrix[0]+omega1*xMatrix[2];
//	AxMatrix[2] = omega2*xMatrix[0]-omega1*xMatrix[1];
//	AxMatrix[3] = alpha*xMatrix[2]-alpha*xMatrix[3];
//	AxMatrix[4] = xMatrix[2];
//
//	for(int i=0; i<5; i++){
//		xdot[i] = AxMatrix[i]+BuMatrix[i]+LCxMatrix[i]+LDvMatrix[i]-LYMatrix[i];
//	}
//
//
//	for(int i=0; i<5;i++){
//		xMatrix[i] = xMatrix[i]+xdot[i]*SENSAMPLE*SAMPLE_TIME;	// difference equation
//	}
//
//#ifdef DEBUGSENSOR_FUSION
////	int dxm[3];
////	for(int i=0; i<3;i++){
////		dxm[i] = xMatrix[i]*1000;		// convert to integer
////	}
////	printf("%5d\r\n",dxm[2]);
////
////	printf("=%d\r\n",dxm[0]);
////	float tt = -atan(xMatrix[1]/xMatrix[0]+ 1e-6);
////	int ptt = tt*(180/PI)*1000;
////	printf("%d\r\n",ptt);
//	int og = Theta_dot*1000;
////	int se = BuMatrix[2]*1000;
////	int ff = LDvMatrix[2]*1000;
////	printf("%d,%d,%d\r\n",og,se,ff);
//		printf("%d,",og);
////	int ptt = Theta*(180/PI)*1000;
////	int pop = omega3*(180/PI)*1000;//Theta_dot*(180/PI)*1000;
////	printf("%d\r\n",ptt);
//#endif
//
//	Psi = -asin(-xMatrix[2]/GRAVITY);
//	Theta = -atan(xMatrix[1]/(xMatrix[0] + 1e-6));
//	PhiDot = (omega1*cos(Theta)-omega2*sin(Theta))/cos(Psi);
//
//	Theta_dot = omega3 - PhiDot*sin(Psi);
//		int og1 = Theta*(180/PI)*10;	// 2/10 help
//		printf("%d\r\n",og1);
//
////
//	////////////////////////////////////// matrix calculation
//}
