#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "HTmotor.h"
#include "UI.h"
#include "Power_limit.h"
#include "supercap.h"
#include "judgement.h"
#include "xuc_can.h"
#include "Heat_limit.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

extern float Kp = 10;
extern float Kd = 0.6;
extern int start_flag;
#define CTRL_PERIOD_MS          10      // 控制周期，可按实际循环频率调整
#define JAM_CURRENT_THRESHOLD   9000    // 卡弹电流阈值，需结合实机调参
#define JAM_DETECT_TIME_MS      1500    // 超阈值持续该时间后判定卡弹
#define JAM_BACK_TIME_MS        2100    // 卡弹后反转持续时间
void TASK::Init()
{
   // 创建启动任务并启动调度器
	xTaskCreate((TaskFunction_t)start_task,            // Task entry
        (const char*)"start_task",          // 任务入口
		(uint16_t)START_STK_SIZE,        // 任务名
		(void*)NULL,                  // 栈大小
		(UBaseType_t)START_TASK_PRIO,       // 参数
		(TaskHandle_t*)&StartTask_Handler);   // 任务句柄
	vTaskStartScheduler();          // 启动 FreeRTOS 调度器
}

/*
创建所有应用任务
*/
void start_task(void* pvParameters)
{
  taskENTER_CRITICAL();           // 进入临界区
	// 创建各任务

	xTaskCreate((TaskFunction_t)ArmTask,
		(const char*)"ArmTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);

	xTaskCreate((TaskFunction_t)UiSendTask,
		(const char*)"UiSendTask",
		(uint16_t)UISEND_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)UISEND_TASK_PRIO,
		(TaskHandle_t*)&UiSendTask_Handler);

  vTaskDelete(StartTask_Handler); // 删除启动任务
	taskEXIT_CRITICAL();            // 退出临界区
}
int CNT = 0;
int CNT_1 = 0;
int CNT_2 = 0;
void MotorUpdateTask(void* pvParameters)
{
	
	while (1)
	{
	TickType_t xlastWakeTime = xTaskGetTickCount();
	
		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);

		for (auto& motor : can2_motor)motor.Ontimer(can2.data, can2.temp_data);

			powerLimiter.ApplyToMotors(can1);

        for (int i = 0; i < 3; ++i) {             // 解算前 3 个关节（ID1~ID3）
			DMmotor[i].State_Decode(can2, can2.jointidata);
			DMmotor[i].DMmotor_Ontimer(can2, DMmotor[i].Kp, DMmotor[i].Kd, can2.jointpdata[i]);
		}

		CNT_1++;

  vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));// 任务周期 2ms
}
}

void CanTransimtTask(void* pvParameters)
{
	TickType_t xlastWakeTime = xTaskGetTickCount();
	while (true)
	{
        // 单次循环完成全部发送
		DMmotor[0].DMmotor_transmit();
		DMmotor[1].DMmotor_transmit();
		DMmotor[2].DMmotor_transmit();
		can1.Transmit(0x1ff, can1.temp_data + 8);
		can2.Transmit(0x1ff, can2.temp_data + 8);

		can1.Transmit(0x200, can1.temp_data);
		can2.Transmit(0x200, can2.temp_data);
		
		vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));
	}
}

uint8_t flag_shoot = 0;
void ControlTask(void* pvParameters)
{
	static int fire_hold_cnt = 0;
	static uint16_t jam_current_cnt = 0;    // 持续过流计数
	static uint16_t jam_back_cnt = 0;       // 反转计数

	while (true)
	{
		const float judgeLimit = static_cast<float>(judgement.data.robot_status_t.chassis_power_limit);

		if (flag_shoot==1) {
			powerLimiter.SetMaxPower(judgeLimit + 150.0f);
		}
		else if (flag_shoot == 2) {
			powerLimiter.SetMaxPower(judgeLimit - 10.0f);
		}
		else {
			powerLimiter.SetMaxPower(judgeLimit);
		}

		xuc.Encode();
		rc.Update();

		heatLimiter.Update();
		
		if (fabs(can2_motor[0].curspeed) > 5000 && fabs(can2_motor[1].curspeed) > 5000) {

			if (((rc.pc.press_r == 1 && xuc.fire_auto == 1)|| rc.pc.press_l == 1) || rc.rc.go_up == 1
				|| (ctrl.mode == CONTROL::FIRE_CONTROL && rc.rc.ch[2] > 330)) {
              fire_hold_cnt = 30;   // 收到开火请求后保持一段时间
			}

            // 1) 若已处于卡弹反转状态，优先保持反转
			if (ctrl.shooter.jam_block) {
				can1_motor[6].setspeed = -1500;
				jam_back_cnt++;

				if (jam_back_cnt >= (JAM_BACK_TIME_MS / CTRL_PERIOD_MS)) {
					ctrl.shooter.jam_block = false;
					jam_back_cnt = 0;
					jam_current_cnt = 0;
					can1_motor[6].setspeed = 0;
				}
			}
          // 2) 未卡弹时，在保持窗口内执行正转供弹
			else if (fire_hold_cnt > 0 && heatLimiter.CanFire()) {
				fire_hold_cnt--;

				can1_motor[6].setspeed = static_cast<int32_t>(6000.f * heatLimiter.GetFireSpeedScale());

              // 检查电机电流是否超过卡弹阈值
				if (can1_motor[6].current > JAM_CURRENT_THRESHOLD ||
					can1_motor[6].current < -JAM_CURRENT_THRESHOLD) {
					jam_current_cnt++;
				}
				else {
					jam_current_cnt = 0;
				}

                // 过流持续足够时间后进入卡弹反转模式
				if (jam_current_cnt >= (JAM_DETECT_TIME_MS / CTRL_PERIOD_MS)) {
					ctrl.shooter.jam_block = true;
					jam_back_cnt = 0;
					jam_current_cnt = 0;
					can1_motor[6].setspeed = -1500;
				}
			}
            // 3) 无开火请求时停止拨盘电机
			else {
				if (fire_hold_cnt > 0) fire_hold_cnt--;
				can1_motor[6].setspeed = 0;
				jam_current_cnt = 0;
			}
		}
		else
		{
			can1_motor[6].setspeed = 0;
			fire_hold_cnt = 0;
			jam_current_cnt = 0;
			jam_back_cnt = 0;
			ctrl.shooter.jam_block = false;
		}
		ctrl.chassis.Update();
		ctrl.pantile.Update();
		ctrl.shooter.Update();
		vTaskDelay(2);
	}
}




void DecodeTask(void* pvParameters)
{
	while (true)
	{
		//rc.Decode_NEW();
		rc.Decode();
		supercap.decode();
		judgement.BuffData();
		judgement.GetData();
		imu_pantile.Decode();
		xuc.Decode();
		vTaskDelay(1);
	}
}

// 优先级 2 —— UI 发送 + 超级电容编码，不受低优先级 CAN 操作阻塞
void UiSendTask(void* pvParameters)
{
	static uint8_t b_last = 0;

	while (true)
	{
		supercap.encode();

		if (rc.pc.B == 1 && b_last == 0) {
			rc.judement_start = true;
			ui.count = 0;
			ui.graphInit = 0;
		}
		b_last = rc.pc.B;

		if (rc.judement_start) {
			judgement.SendData();
		}

		// --------------------- G键达妙直接发使能 ----------------------
		if (rc.pc.G == 1 && rc.g_last == 0)
		{
			rc.g_toggle = !rc.g_toggle;
		}
		rc.g_last = rc.pc.G;

		if (rc.g_toggle) {
			for (uint32_t i = 0; i < 3; ++i) {
				DMmotor[i].Motor_Start(can2, DMmotor[i].GetControlStdId());
			}
		}

		vTaskDelay(30);
	}
}

// 优先级 1 —— DM 电机使能，可能阻塞在 CAN 发送（被 UiSendTask 抢占）
void ArmTask(void* pvParameters)
{
	static uint8_t enable_key_last = 0;
	static uint8_t dm_reenable_pending = 0;
	static uint8_t dm_reenable_retry = 0;
	static TickType_t dm_reenable_tick = 0;

	while (true)
	{
		const uint8_t enable_key_pressed = (rc.pc.CTRL == 1 || rc.rc.custom_key_l == 1) ? 1U : 0U;
		const TickType_t now = xTaskGetTickCount();

		if (enable_key_pressed == 1 && enable_key_last == 0) {
			ctrl.init_DM = 0;
			dm_reenable_pending = 1;
			dm_reenable_retry = 0;
			dm_reenable_tick = now + pdMS_TO_TICKS(800);
		}

		if (dm_reenable_pending == 1 && now >= dm_reenable_tick) {
			uint8_t has_fault = 0;
			uint8_t need_start = 0;

			for (uint32_t i = 0; i < 3; ++i) {
				const uint8_t err = DMmotor[i].err;
				if (err >= 8) {
					has_fault = 1;
					DMmotor[i].CanComm_ControlCmd(can2, CMD_CLEAR_MODE, DMmotor[i].GetControlStdId());
				} else if (err == 0) {
					need_start = 1;
				}
			}

			if (has_fault == 0) {
				for (uint32_t i = 0; i < 3; ++i) {
					if (DMmotor[i].err == 0) {
						DMmotor[i].Motor_Start(can2, DMmotor[i].GetControlStdId());
					}
				}
			}

			if (dm_reenable_retry == 0) {
				ctrl.init_DM = 1;
			}

			dm_reenable_retry++;
			if ((has_fault == 0 && need_start == 0) || dm_reenable_retry >= 20) {
				dm_reenable_pending = 0;
			} else {
				dm_reenable_tick = now + pdMS_TO_TICKS(100);
			}
		}

		enable_key_last = enable_key_pressed;
		vTaskDelay(30);
	}
}





