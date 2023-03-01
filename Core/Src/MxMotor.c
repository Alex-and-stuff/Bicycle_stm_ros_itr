/*
 * MxMotor.c
 *
 *  Created on: 2019¦~11¤ë27¤é
 *      Author: LDSCB
 *
 *      brief
 *    	code only protocol1 Mx106
 *    	you have to setup Mx motor offline
 */



//#define DEBUGMX 1

#include "MxMotor.h"

volatile uint8_t gbpRxInterruptBuffer[256]; // dxl buffer
volatile uint8_t gbRxBufferWritePointer, gbRxBufferReadPointer;
volatile uint32_t gwTimingDelay, gw1msCounter;

volatile uint16_t CCR1_Val = 10;		// 1ms_100	1=10us
volatile uint32_t capture = 0;

uint16_t GoalPos = MXHEX_CENTER;		// angle range form 0x000 to 0xFFF 180 836
uint16_t Position;
uint16_t wPresentPos;
uint16_t wPresentSpd;
uint8_t Voltage;
uint8_t bMoving, CommStatus;
uint8_t Rx;

int jk=0;
int sn = 200;
int16_t step[500]={0};

void ClearBuffer256(void) {
	gbRxBufferReadPointer = gbRxBufferWritePointer = 0;
}

uint8_t CheckNewArrive(void) {
	if (gbRxBufferReadPointer != gbRxBufferWritePointer)
		return 1;
	else
		return 0;
}

void TxDByte_DXL(uint8_t bTxdData) {

	LL_USART_IsActiveFlag_TC(UART4);
	LL_USART_TransmitData8(UART4, bTxdData);

	while (LL_USART_IsActiveFlag_TC(UART4) == RESET) {
	}

// Back up for USART HALF to plex
//	LL_USART_TransmitData8(UART4, bTxdData);
//
//	while (LL_USART_IsActiveFlag_TXE(UART4) == RESET) {
//	}

//	printf("bTxData=%x\r\n",bTxdData);
	//printf("TT\r\n");
}

uint8_t RxDByte_DXL(void) {
	uint8_t bTemp;

	while (1) {
		if (gbRxBufferReadPointer != gbRxBufferWritePointer)
			break;
	}

	bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];

	gbRxBufferReadPointer++;
//	printf("btemp=%d\r\n",bTemp);
	return bTemp;
}

void PrintCommStatus(int CommStatus) {
#ifdef DEBUGMX
	switch (CommStatus) {
	case COMM_TXFAIL:
		TxDString("COMM_TXFAIL: Failed transmit instruction packet!\r\n");
		break;

	case COMM_TXERROR:
		TxDString("COMM_TXERROR: Incorrect instruction packet!\r\n");
		break;

	case COMM_RXFAIL:
		TxDString("COMM_RXFAIL: Failed get status packet from device!\r\n");
		break;

	case COMM_RXWAITING:
		TxDString("COMM_RXWAITING: Now recieving status packet!\r\n");
		break;

	case COMM_RXTIMEOUT:
		TxDString("COMM_RXTIMEOUT: There is no status packet!\r\n");
		break;

	case COMM_RXCORRUPT:
		TxDString("COMM_RXCORRUPT: Incorrect status packet!\r\n");
		break;

	default:
		TxDString("This is unknown error code!\r\n");
		break;
	}
#endif
}

// Print error bit of status packet
void PrintErrorCode() {
#ifdef DEBUGMX
	if (dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		TxDString("Input voltage error!\r\n");

	if (dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		TxDString("Angle limit error!\r\n");

	if (dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		TxDString("Overheat error!\r\n");

	if (dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		TxDString("Out of range error!\r\n");

	if (dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		TxDString("Checksum error!\r\n");

	if (dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		TxDString("Overload error!\r\n");

	if (dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		TxDString("Instruction code error!\r\n");
#endif
}
///////////////////////////////// computer
void TxDString(uint8_t *bData) {
	while (*bData)
		TxDByte_PC(*bData++);
}

void TxDWord16(uint16_t wSentData) {
	TxDByte16((wSentData >> 8) & 0xff);
	TxDByte16(wSentData & 0xff);
}

void TxDByte16(uint8_t bSentData) {
	uint8_t bTmp;

	bTmp = ((uint8_t) (bSentData >> 4) & 0x0f) + (uint8_t) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
	bTmp = (uint8_t) (bSentData & 0x0f) + (uint8_t) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
}

void TxDByte_PC(uint8_t bTxdData) {
	LL_USART_TransmitData8(USART2, bTxdData);
	while (LL_USART_IsActiveFlag_TXE(USART2) == RESET);
}
///////////////////////////////// computer

void TimerInterrupt_1ms(void) //OLLO CONTROL
{
	if (LL_TIM_IsActiveFlag_CC1(TIM2) != RESET) // decided by CCR1_Val
			{
		capture = LL_TIM_OC_GetCompareCH1(TIM2);
		LL_TIM_OC_SetCompareCH1(TIM2, capture + CCR1_Val);

		if (gw1msCounter > 0){
			gw1msCounter--;
		}
		LL_TIM_ClearFlag_CC1(TIM2);
	}
}

void RxD0Interrupt(void) {

	if (LL_USART_IsEnabledIT_RXNE(UART4) != RESET) {
		gbpRxInterruptBuffer[gbRxBufferWritePointer++] = LL_USART_ReceiveData8(UART4);
	}
}

void mDelay(uint32_t nTime) {
	HAL_Delay(nTime);
}

void StartDiscount(int32_t StartTime) {
	gw1msCounter = StartTime;
}

uint8_t CheckTimeOut(void) {
	// Check timeout
	// Return: 0 is false, 1 is true(timeout occurred)

	if (gw1msCounter == 0)
		return 1;
	else
		return 0;
}

//////////////////////////
void Motor() {	//test function , have to define DEBUGMX in line 8 to know MxMotor Status

	dxl_initialize(0, 1);	// just remind you to setup the Mxmotor baud rate

	bMoving = dxl_read_byte(MOTOR_ID, P_MOVING);	//check the motor is moving or not; less than 200us Tx+Rx
	CommStatus = dxl_get_result();
	if (CommStatus == COMM_RXSUCCESS) {
		if (bMoving == 0) {

			// Write goal position
			dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, GoalPos);	//it is position moving //GoalPos
		}

		PrintErrorCode();

		// Read present position
		wPresentPos = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
		TxDWord16(GoalPos);		//bit4~bit7 is current position
		//TxDString("   ");
		TxDWord16(wPresentPos);
		TxDByte_PC('\r');
		TxDByte_PC('\n');
	} else
		PrintCommStatus(CommStatus);
}

void PIDsetup(int Pgain,int Igain, int Dgain){
	dxl_write_byte(MOTOR_ID, P_GAIN, Pgain);
	dxl_write_byte(MOTOR_ID, I_GAIN, Igain);
	dxl_write_byte(MOTOR_ID, D_GAIN, Dgain);
}

void AngleLimit(uint16_t upper_bound, uint16_t lower_bound){
	dxl_write_word(MOTOR_ID, CCW_LIMIT, upper_bound);
	dxl_write_word(MOTOR_ID, CW_LIMIT , lower_bound);
}

void Torque_off(){
	dxl_write_byte(MOTOR_ID, Torque_Enable, 0);
}
void Torque_on(){
	dxl_write_byte(MOTOR_ID, Torque_Enable, 1);
}

uint8_t TorqueState(){
	return dxl_read_byte(MOTOR_ID,Torque_Enable);
}

#ifdef DEBUGMX

void ttst(){
	dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, GoalPos);
	for(jk=0;jk<500;jk++){
		step[jk] =dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
	}
	bMoving = dxl_read_byte(MOTOR_ID, P_MOVING);	//check the motor is moving or not;
	while(bMoving){
		bMoving = dxl_read_byte(MOTOR_ID, P_MOVING);
	}
}

void pp(){
	for(jk=0;jk<500;jk++){
		printf("%dpo=%08x\r\n",jk,step[jk]);
	}
}


void MotorResponse(uint16_t testangle){
	int position_array[10000];
	int i = 0;
	uint8_t moving = 1;
	uint16_t nowposition;
	dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, MXSTOP_CENTER+testangle);
	LL_GPIO_TogglePin(GPIOA, LD2_Pin);
	while(moving == 1){
		nowposition = dxl_read_word(MOTOR_ID, P_PRESENT_POSITION_L);
		position_array[i] = nowposition;
		++i;
		moving = dxl_read_byte(MOTOR_ID, P_MOVING);
	}
	LL_GPIO_TogglePin(GPIOA, LD2_Pin);
	for(int j=0;j<=i;++j){
		printf("%d\r\n");
	}
}

void MxReadSpeed(){
	dxl_write_word(MOTOR_ID, P_GOAL_POSITION_L, GoalPos-0x200);
	for(jk=0;jk<500;jk++){
		step[jk] = dxl_read_word(MOTOR_ID, P_PRESENT_SPEED_L);
	}
}
#endif

