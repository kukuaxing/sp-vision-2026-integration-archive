#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*
头文件中不要进行全局变量定义，可进行宏定义
全局变量用extern声明，避免头文件多次引用multiple defined
*/

// ===== [后续修改开始] 当前实车硬件数量与队列映射 =====
#define CAN1_MOTOR_NUM 3 // [后续修改] CAN1接2个3508摩擦轮(ID1/ID4)和1个2006拨弹轮(ID5)；达妙pitch电机由DMMOTOR单独管理
#define CAN2_MOTOR_NUM 5 // [后续修改] CAN2接4个3508底盘(ID1~ID4，反馈0x201~0x204)和1个6020 yaw(反馈0x208)


// ===== [Next] shooter test params: 8-slot feeder + M2006 (36:1) =====
#define FEEDER_SLOTS 8
#define M2006_GEAR_RATIO 36.0f
#define ENCODER_TICKS_PER_REV 8192
// one bullet = 1/8 output rev = 4.5 rotor revs = 4.5*8192 = 36864 encoder ticks
#define BULLET_FEED_TICKS ((int32_t)(ENCODER_TICKS_PER_REV * M2006_GEAR_RATIO / FEEDER_SLOTS))
#define WHEELS_READY_RATIO 0.90f // start feeding only after wheels reach 90% speed
#define WHEELS_MIN_RATIO 0.70f   // pause feeding while wheels stay below 70%
#define JAM_SPEED_RATIO 0.30f    // supply speed below this ratio means jam
#define JAM_TIMEOUT_MS 80        // jam detection time
#define JAM_RECOVER_MS 150       // jam recovery time
#define MAX_JAMS_PER_SHOT 3      // max jams before protection stops the shot
#define RC_LINK_TIMEOUT_MS 100   // stop shooting when DR16 link is lost

#define CHASSIS_MOTOR_NUM 4 // 底盘为4轮全向轮结构
#define PANTILE_MOTOR_NUM 1 // 当前只控制yaw轴，未接pitch电机

#define PANTILE_IMU_ENABLE 1 // [后续修改] 启用云台IMU参与yaw锁角
#define SHOOTER_MOTOR_NUM 2
#define SUPPLY_MOTOR_NUM 1

#define RcQueueHandle Uart3QueueHandler // [后续修改] DR16遥控器数据来自USART3队列
#define ImuQueueHandle Uart2QueueHandler // [后续修改] CH010陀螺仪数据来自USART2队列
#define JudgementQueueHandle Uart5QueueHandler // 电源/裁判相关串口暂用UART5队列
// ===== [后续修改结束] 当前实车硬件数量与队列映射 =====

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
#define CONTROL_STK_SIZE 		128  
extern TaskHandle_t ControlTask_Handler;

#define MOTOR_TASK_PRIO		2
#define MOTOR_STK_SIZE 		256  
extern TaskHandle_t MotorTask_Handler;

#define CANTX_TASK_PRIO		2
#define CANTX_STK_SIZE 		256 
extern TaskHandle_t CanTxTask_Handler;

#define XUC_TASK_PRIO        2
#define XUC_STK_SIZE         256
extern TaskHandle_t XucTask_Handler;

extern PARAMETER para;
