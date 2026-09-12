#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "label.h"

void PARAMETER::Init()
{
	pitch_min = 0, pitch_max = 8192, initial_pitch = 4096, initial_yaw = 6353;
	imu_pitch_max = 18, imu_pitch_min = 16;
	ace_speed = 1000, max_speed = 3500, rota_speed = 4500;
	pitch_speed = 2, yaw_speed = 2;
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
TaskHandle_t UiSendTask_Handler;

