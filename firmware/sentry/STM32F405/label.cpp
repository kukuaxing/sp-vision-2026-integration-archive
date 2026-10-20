#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "label.h"

void PARAMETER::Init()
{
	pitch_min = 0, pitch_max = 8192, initial_pitch = 4096, initial_yaw = 4096;
	imu_pitch_max = 18, imu_pitch_min = 16;
	ace_speed = 1000, max_speed = 3000, rota_speed = 1000;
	pitch_speed = 2, yaw_speed = 2;
	dm_initial_pitch = 0.f, dm_initial_yaw = 0.f;
	yaw_center = 8192.f, yaw_span = 1707.f;//左右75度大概
	bullet_step = 42130.29f;bullet_lead_max = 84260.58f;
	dm_pitch_min = -0.55f;   
	dm_pitch_max = +0.4f;  
	// 底盘导航：视觉速度 → rpm
	DV_MAX = 200.f;                    // 每周期(5ms)最多变 200rpm
	NAV_MAX = 4000.f;                  // 与摇杆量程对齐
	K_NAV = 159.f;           // 速度增益
}


/*
定义任务句柄
*/
TaskHandle_t StartTask_Handler;
TaskHandle_t LedTask_Handler;
TaskHandle_t DecodeTask_Handler;
TaskHandle_t ControlTask_Handler;
TaskHandle_t MotorTask_Handler;
TaskHandle_t CanTxTask_Handler;

