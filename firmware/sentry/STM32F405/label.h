#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*
头文件中不要进行全局变量定义，可进行宏定义
全局变量用extern声明，避免头文件多次引用multiple defined
*/

// ================================================================
// 机器人控制模式选择
// 0: 遥控器控制 (RC)    1: 自动控制 (AUTO)
// ================================================================
#define ROBOT_CONTROL_MODE 0

#define ROBOT_RC_MODE   0
#define ROBOT_AUTO_MODE 1

#define CAN1_MOTOR_NUM 4
#define CAN2_MOTOR_NUM 4

#define CHASSIS_MOTOR_NUM 4
#define PANTILE_MOTOR_NUM 2
#define SHOOTER_MOTOR_NUM 2
#define SUPPLY_MOTOR_NUM 1

#define RcQueueHandle Uart2QueueHandler
#define ImuQueueHandle Uart5QueueHandler
#define JudgementQueueHandle Uart3QueueHandler

#define degreeToMechanical(a) ((a)*8192.f/360.f)
#define mechanicalToDegree(a) ((a)*360.f/8192.f)
#define degreeToRadian(a) ((a)*3.14159265f/180.f)
#define radianToDegree(a) ((a)*180.f/3.14159265f)
#ifndef PI
#define PI 3.14159265f
#endif // !PI


class PARAMETER
{
public:

	float pitch_max{}, imu_pitch_max{}, pitch_min{}, imu_pitch_min{}, orgin_pitch{}, initial_pitch{}, initial_yaw{};
	int32_t ace_speed{}, max_speed{}, rota_speed{};
	int32_t pitch_speed{}, yaw_speed{};

	PARAMETER& Init();


};


//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小
#define START_STK_SIZE 		128
//任务句柄
extern TaskHandle_t StartTask_Handler;
//任务函数

#define LED_TASK_PRIO		1
#define LED_STK_SIZE 		128
extern TaskHandle_t LedTask_Handler;

#define DECODE_TASK_PRIO		3
#define DECODE_STK_SIZE 		128
extern TaskHandle_t DecodeTask_Handler;

#define RC_TASK_PRIO		3
#define RC_STK_SIZE 		256
extern TaskHandle_t RcTask_Handler;

#define CONTROL_TASK_PRIO		2
#define CONTROL_STK_SIZE 		256
extern TaskHandle_t ControlTask_Handler;

#define MOTOR_TASK_PRIO		2
#define MOTOR_STK_SIZE 		256
extern TaskHandle_t MotorTask_Handler;

#define CANTX_TASK_PRIO		2
#define CANTX_STK_SIZE 		256
extern TaskHandle_t CanTxTask_Handler;

extern PARAMETER para;
