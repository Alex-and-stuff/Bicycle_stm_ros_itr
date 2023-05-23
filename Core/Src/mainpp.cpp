/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

ros::NodeHandle nh;

geometry_msgs::Twist vel_cmd;
geometry_msgs::Twist pub_state;
geometry_msgs::Twist pub_gps;

ros::Publisher state_pub("/output", &pub_state);
ros::Publisher gps_pub("/stm_gps", &pub_gps);

void command_cb(const geometry_msgs::Twist& msg){
	vel_cmd = msg;
}

ros::Subscriber<geometry_msgs::Twist> vel_cmd_sub("/cmd_vel", &command_cb);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}


void setup(void)
{
  nh.initNode();
  nh.advertise(state_pub);
  nh.advertise(gps_pub);
  nh.subscribe(vel_cmd_sub);
}

uint32_t tickstart = 0;
void loop(int* val)
{

	if (nh.connected()){
		pub_state.linear.x = *val;
		state_pub.publish(&pub_state);
	}
	nh.spinOnce();
}

void loop2(float* theta, float* theta_dot, float* delta, float* v, float* cmd, float acc_x,
		float position_omega, double* lat_current, double* lon_current, float* horizontal_accuracy, float* vertical_accuracy, float* point_current){

//	nh.setSpinTimeout(10);
	if (nh.connected()){
		pub_state.linear.x = *theta;
		pub_state.linear.y = *theta_dot;
		pub_state.linear.z = *delta;
		pub_state.angular.x = *v;
		pub_state.angular.y = acc_x;  // acceleration
		pub_state.angular.z = position_omega;   // phi_dot

		pub_gps.linear.x = *lat_current;
		pub_gps.linear.y = *lon_current;
		pub_gps.linear.z = *horizontal_accuracy;
		pub_gps.angular.x = point_current[0];
		pub_gps.angular.y = point_current[1];
		pub_gps.angular.z = *vertical_accuracy;

		cmd[0] = vel_cmd.linear.x;
		cmd[1] = vel_cmd.linear.y;

		state_pub.publish(&pub_state);
		gps_pub.publish(&pub_gps);
	}
	nh.spinOnce();
	HAL_Delay(10);
}

