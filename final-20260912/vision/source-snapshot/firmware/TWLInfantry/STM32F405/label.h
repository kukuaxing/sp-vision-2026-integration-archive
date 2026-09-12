#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*
头文件中不要进行全局变量定义，可进行宏定义
全局变量用extern声明，避免头文件多次引用multiple defined
*/

#define CAN1_MOTOR_NUM 8
#define CAN2_MOTOR_NUM 6

#define CHASSIS_MOTOR_NUM 6
#define PANTILE_MOTOR_NUM 1
#define SHOOTER_MOTOR_NUM 2
#define SUPPLY_MOTOR_NUM 1

#define RcQueueHandle Uart2QueueHandler
#define ImuQueueHandle Uart5QueueHandler
#define JudgementQueueHandle Uart3QueueHandler

#define degreeToMechanical(a) ((a)*8192.f/360.f)
#define mechanicalToDegree(a) ((a)*360.f/8192.f)

#ifndef PI
#define PI 3.1415926
#endif // !PI


class PARAMETER
{
public:

	float pitch_max{}, imu_pitch_max{}, pitch_min{}, imu_pitch_min{}, orgin_pitch{}, initial_pitch{}, initial_yaw{};
	int32_t ace_speed{}, max_speed{}, rota_speed{};
	int32_t pitch_speed{}, yaw_speed{};

	void Init();


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

#define DECODE_TASK_PRIO		5
#define DECODE_STK_SIZE 		128  
extern TaskHandle_t DecodeTask_Handler;

#define RC_TASK_PRIO		2
#define RC_STK_SIZE 		256  
extern TaskHandle_t RcTask_Handler;

#define CONTROL_TASK_PRIO		4
#define CONTROL_STK_SIZE 		128  
extern TaskHandle_t ControlTask_Handler;

#define MOTOR_TASK_PRIO		3
#define MOTOR_STK_SIZE 		256  
extern TaskHandle_t MotorTask_Handler;

#define CANTX_TASK_PRIO		2
#define CANTX_STK_SIZE 		256 
extern TaskHandle_t CanTxTask_Handler;

#define UISEND_TASK_PRIO		2
#define UISEND_STK_SIZE 		256
extern TaskHandle_t UiSendTask_Handler;

extern PARAMETER para;