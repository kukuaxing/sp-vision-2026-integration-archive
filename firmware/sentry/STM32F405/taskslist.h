#pragma once
#include "FreeRTOS.h"
#include "task.h"


class TASK
{
public:
	void Init();
	bool led_flag = false;
	uint32_t counter;
private:

};
void start_task(void* pvParameters);
void CanTransimtTask(void* pvParameters);
void ArmTask(void* pvParameters);
void DecodeTask(void* pvParameters);
void MotorUpdateTask(void* pvParameters);
void ControlTask(void* pvParameters);

extern TASK task;
