#include "LCD.h"
#include "math.h"

void LcdData(uint8_t* tx_data, float* mu, float* accuracy, float* point_current, float V_current,
			float theta_x, float theta_y, uint8_t tracking_first_flag){
	/* @input:    rx_data: array storing data to be transmitted
	 * @input:    mu: EKF state[X, Y, phi, v0]
	 * @input:    accuracy: GPS horizontal accuracy
	 * @input:    point_current: GPS position
	 * @input:    V_current: current velocity (encoder data)
	 * @input:    theta_x: roll angle
	 * @function: built TX data to be transmitted to stm2 LCD display
	 * */

	// add start_bit to ensure we receive at the correct position
	tx_data[START] = START_BIT;

	// EKF fused state
	int16_t lcd_ekfx   = mu[0] * LCD_GAIN;
	int16_t lcd_ekfy   = mu[1] * LCD_GAIN;
	int16_t lcd_ekfphi = mu[2] * LCD_GAIN;

	tx_data[EXFx_f] = lcd_ekfx >> 8U;
	tx_data[EKFx_s] = lcd_ekfx;
	tx_data[EKFy_f] = lcd_ekfy >> 8U;
	tx_data[EKFy_s] = lcd_ekfy;
	tx_data[EKFphi_f] = lcd_ekfphi >> 8U;
	tx_data[EKFphi_s] = lcd_ekfphi;

	// GPS horizontal accuracy
	int16_t lcd_acc   = accuracy[0] * LCD_GAIN;

	tx_data[GPSacc_f] = lcd_acc >> 8U;
	tx_data[GPSacc_s] = lcd_acc;

	// GPS position
	int16_t lcd_gpsx  = point_current[0] * LCD_GAIN;
	int16_t lcd_gpsy  = point_current[1] * LCD_GAIN;

	tx_data[GPSx_f] = lcd_gpsx >> 8U;
	tx_data[GPSx_s] = lcd_gpsx;
	tx_data[GPSy_f] = lcd_gpsy >> 8U;
	tx_data[GPSy_s] = lcd_gpsy;

	// Bicycle current velocity
	int16_t lcd_vel   = V_current * LCD_GAIN;

	tx_data[VEL_f] = lcd_vel >> 8U;
	tx_data[VEL_s] = lcd_vel;

	// Tilt angle display
	if (tracking_first_flag == 0){
		if (theta_x == 0 && theta_y == 0){                             // if no IMU data is received, make all three lights light up
			tx_data[IMU] = LED_ALL;
		}else{
			double angle_margin = 0.02;
			if (theta_x > (0.0 + angle_margin)){                       // if the bicycle is about horizontal, light up the red LED
				tx_data[IMU] = LED_LEFT;
			}
			else if (theta_x < (0.0 - angle_margin)){                  // if the bicycle is tilted left (about x-axis), light up the left green LED
				tx_data[IMU] = LED_RIGHT;
			}
			else{                                                      // if the bicycle is tilted right (about x-axis), light up the right green LED
				tx_data[IMU] = LED_MID;
			}
		}
	}else{
		if (theta_x == 0 && theta_y == 0){                             // if no IMU data is received, make all three lights light up
			tx_data[IMU] = LED_ALL;
		}else{
			double angle_margin = 0.02;
			if (theta_x > (0.0 + angle_margin)){                       // if the bicycle is about horizontal, light up the red LED
				tx_data[IMU] = LED_NLEFT;
			}
			else if (theta_x < (0.0 - angle_margin)){                  // if the bicycle is tilted left (about x-axis), light up the left green LED
				tx_data[IMU] = LED_NRIGHT;
			}
			else{                                                      // if the bicycle is tilted right (about x-axis), light up the right green LED
				tx_data[IMU] = LED_NMID;
			}
		}
	}

}
