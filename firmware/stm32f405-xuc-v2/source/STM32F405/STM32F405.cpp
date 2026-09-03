  /*
   *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\____________/\\\\\\\_____/\\\\\\\\\\\___
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
#include <vector>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "Power_read.h"
#include "HTmotor.h"
#include "xuc.h"

// ===== [后续修改] 实车电机映射：CAN1摩擦轮+拨弹(+达妙pitch)，CAN2底盘+yaw =====
Motor can1_motor[CAN1_MOTOR_NUM] = { // [后续修改] CAN1挂载2个3508摩擦轮(ID1/ID4)和1个2006拨弹轮(ID5)；达妙pitch电机由DMMOTOR单独管理
	Motor(M3508, SPD, shooter, ID1, PID(4.f, 0.0f, 1.5f, 0.f)), // 摩擦轮1号3508，反馈ID 0x201
	Motor(M3508, SPD, shooter, ID4, PID(4.f, 0.0f, 1.5f, 0.f)), // 摩擦轮2号3508，反馈ID 0x204
	Motor(M2006, SPD, supply, ID5, PID(4.f, 0.0f, 1.5f, 0.f))  // 拨弹轮2006，反馈ID 0x205，指示灯每秒闪5次
};

Motor can2_motor[CAN2_MOTOR_NUM] = { // [后续修改] CAN2挂载4个3508底盘(ID1~ID4)和1个6020 yaw(0x208)
	Motor(M3508, SPD, chassis, ID1, PID(4.f, 0.0f, 1.5f, 0.f)), // 底盘1号3508，反馈ID 0x201
	Motor(M3508, SPD, chassis, ID2, PID(4.f, 0.0f, 1.5f, 0.f)), // 底盘2号3508，反馈ID 0x202
	Motor(M3508, SPD, chassis, ID3, PID(4.f, 0.0f, 1.5f, 0.f)), // 底盘3号3508，反馈ID 0x203
	Motor(M3508, SPD, chassis, ID4, PID(4.f, 0.0f, 1.5f, 0.f)), // 底盘4号3508，反馈ID 0x204
	Motor(M6020, POS, pantile, ID8, PID(50.f, 0.0f, 5.0f, 0.f), PID(1.2f, 0.0f, 8.0f, 0.f)) // 6020 yaw，反馈ID 0x208，指示灯每秒闪4次，运行时在锁角/自由yaw中切到速度模式
};

// ===== [后续修改结束] 实车电机映射 =====
DMMOTOR DMmotor[1] = { DMMOTOR(DM_ID3, P_S, PITCH) }; // [后续修改] 达妙pitch电机J4310，ID=3，位置-速度模式，反馈ID 0x03

CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;


int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();
// ===== [后续修改开始] 实车通信映射：IMU接USART2，DR16接USART3 =====
	can1.Init(CAN1); // 初始化CAN1：2个3508摩擦轮电机和1个2006拨弹轮电机和1个达妙 pitch电机
	//HAL_CAN_Start(&hcan1);
	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can2.Init(CAN2); // 初始化CAN2：yaw轴6020电机和4个3508底盘电机
	timer.Init(BASE, TIM3, 1000).BaseInit();
	#if PANTILE_IMU_ENABLE
	imu_pantile.Init(&uart2, USART2, 921600, CH010); // [后续修改] CH010陀螺仪接USART2，波特率921600
	#endif
	rc.Init(&uart3, USART3, 100000); // [后续修改] DR16接USART3，遥控器协议波特率100000
// ===== [后续修改结束] 实车通信映射 =====
	power.Init(&uart5, UART5, 9600);
	// XUC V2: USART6 (PC6 TX / PC7 RX) is dedicated to the
	// small-computer XUC V2 link. UART5 remains assigned to the power module.
	// Phase 2A permits only explicitly armed, bounded yaw; pitch/shoot stay isolated.
	xuc.Init(&uart6, USART6, XUC_TJ_BAUD_RATE);

	para.Init();
// ===== [后续修改开始] CONTROL只绑定当前实车实际存在的5个电机 =====
	std::vector<Motor*> control_motors; // 统一收集需要CONTROL管理的电机指针
	control_motors.push_back(&can2_motor[0]); // 加入底盘1号电机
	control_motors.push_back(&can2_motor[1]); // 加入底盘2号电机
	control_motors.push_back(&can2_motor[2]); // 加入底盘3号电机
	control_motors.push_back(&can2_motor[3]); // 加入底盘4号电机
	control_motors.push_back(&can2_motor[4]); // 加入yaw轴6020电机
	control_motors.push_back(&can1_motor[0]); // 加入摩擦轮1号3508
	control_motors.push_back(&can1_motor[1]); // 加入摩擦轮2号3508
	control_motors.push_back(&can1_motor[2]); // 加入拨弹轮2006
	ctrl.Init(control_motors); // 按电机function分类绑定到底盘/云台/发射/拨弹控制数组
// ===== [后续修改结束] CONTROL电机绑定 =====
	delay.delay_ms(1000); // [后续修改] 等待达妙电机上电初始化完成（约500ms~1s），避免使能帧发太早被忽略
	DMmotor[0].DMmotorinit(); // [后续修改] 使能CAN1上的达妙pitch电机（广播位置-速度模式使能帧0xFD）

	task.Init();
	for (;;)
		;
}
