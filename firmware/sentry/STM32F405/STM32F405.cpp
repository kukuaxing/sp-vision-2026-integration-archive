/*
 *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\\\____________/\\\\\\\\\_____/\\\\\\\\\\\___
 * _\///\\\___/\\\/__\/\\\\\\________/\\\\\\_\/\\\_______\/\\\____________/\\\///////\\\_______/\\\////////____/\\\/////////\\\_
 *  ___\///\\\\\\/____\/\\\//\\\____/\\\//\\\_\/\\\_______\/\\\___________\/\\\_____\/\\\_____/\\\/____________\//\\\______\///__
 *   _____\//\\\\______\/\\\\///\\\/\\\/_\/\\\_\/\\\_______\/\\\___________\/\\\\\\\\\\\/_____/\\\_______________\////\\\_________
 *    ______\/\\\\______\/\\\__\///\\\/___\/\\\_\/\\\_______\/\\\___________\/\\\//////\\\____\/\\\__________________\////\\\______
 *     ______/\\\\\\_____\/\\\____\///_____\/\\\_\/\\\_______\/\\\___________\/\\\____\//\\\___\//\\\____________________\////\\\___
 *      ____/\\\////\\\___\/\\\_____________\/\\\_\//\\\______/\\\____________\/\\\_____\//\\\___\///\\\___________/\\\______\//\\\__
 *       __/\\\/___\///\\\_\/\\\_____________\/\\\__\///\\\\\\\\\/_____________\/\\\______\//\\\____\////\\\\\\\\\_\///\\\\\\\\\\\/___
 *        _\///_______\///__\///_____________\///_____\/////////_______________\///________\///________\/////////____\///////////_____
*/

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "power_read.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "DMmotor.h"
#include "DMmotor.h"
#include "nuc.h"
#include "slidingmodec.h"
#include "supercap.h"

Motor can1_motor[CAN1_MOTOR_NUM] = {
	Motor(M3508,SPD,shooter, ID2, PID(1.78f, 0.f, 0.0f)),
	Motor(M3508,SPD,shooter, ID3, PID(1.78f, 0.f, 0.0f)),
	Motor(M2006,SPD,supply,  ID7, PID(1.8f, 0.f, 0.4f)),
	Motor(M6020,POS,pantile,  ID5, PID(40.f, 0.85f, 0.f), PID(5.f, 0.0f, 12.f))
};
Motor can2_motor[CAN2_MOTOR_NUM] = //轮电机+大yaw
{
	Motor(M3508,SPD,chassis, ID1, PID(3.5f, 0.f, 0.05f)),
	Motor(M3508,SPD,chassis, ID2, PID(3.5f, 0.f, 0.05f)),
	Motor(M3508,SPD,chassis, ID3, PID(3.5f, 0.f, 0.05f)),
	Motor(M3508,SPD,chassis, ID4, PID(3.5f, 0.f, 0.05f)),
};




CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile, imu_gimbal;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;
NUC xuc;
SUPERCAP supercap;
DMMOTOR DM_motorYaw = { DMMOTOR(0x06 , &can2) };
DMMOTOR DM_motorPitch = { DMMOTOR(0x09 , &can1) };

int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();
	can1.Init(CAN1);
	//HAL_CAN_Start(&hcan1);
	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();
	imu_pantile.Init(&uart1, USART1, 115200, CH010);
	imu_gimbal.Init(&uart5, UART5, 115200, CH010);
	rc.Init(&uart3, USART3, 100000);
	//power.Init(&uart6, USART6, 9600);
	judgement.Init(&uart2, 115200, USART2);
	supercap.Init(&uart6, 115200, USART6);
	xuc.Init(&uart4, UART4, 460800);
	para.Init();
	ctrl.Init(
		{ &can2_motor[0], &can2_motor[1], &can2_motor[2], &can2_motor[3], //can2[0]~[3]为底盘电机
		   &can1_motor[0], &can1_motor[1], &can1_motor[2],&can1_motor[3]
		//  &can2_motor[4],&can1_motor[3], //yaw pitch
		});
	//DM_motorYaw.MotorStart(0x206);//DM 大Yaw轴电机初始使能，速度模式
//	DM_motorPitch.MotorStart(0x109);//DM Pitch轴电机初始使能
	task.Init();

	for (;;)

		;
}
