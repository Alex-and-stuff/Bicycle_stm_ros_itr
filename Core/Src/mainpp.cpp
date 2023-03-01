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

ros::Publisher output("output", &pub_state);

void command_cb(const geometry_msgs::Twist& msg){
	vel_cmd = msg;
}

ros::Subscriber<geometry_msgs::Twist> vel_cmd_sub("/vel_command", &command_cb);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}


void setup(void)
{
  nh.initNode();
  nh.advertise(output);
  nh.subscribe(vel_cmd_sub);
}

uint32_t tickstart = 0;
void loop(int* val)
{

	if (nh.connected()){
//		str_msg.data = hello;
		pub_state.linear.x = *val;
//		chatter.publish(&str_msg);
		output.publish(&pub_state);
	}
	nh.spinOnce();
}

void loop2(float* theta, float* theta_dot, float* delta, float* v, float* cmd)
{

//	nh.setSpinTimeout(10);
	if (nh.connected()){
		pub_state.linear.x = *theta;
		pub_state.linear.y = *theta_dot;
		pub_state.linear.z = *delta;
		pub_state.angular.x = *v;

		cmd[0] = vel_cmd.linear.x;
		cmd[1] = vel_cmd.linear.y;

		output.publish(&pub_state);
	}
	nh.spinOnce();
	HAL_Delay(10);
}

