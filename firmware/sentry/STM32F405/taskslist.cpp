#include "label.h"
#include "taskslist.h"
#include "calibration_mode.h"
#include "pitch_zero.h"
extern void GimbalDiagnosticUpdate();
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "DMmotor.h"
#include "nuc.h"
#include "PowerLimiter.h" // 引入头文件
#include "judgement.h"    // 读取裁判系统功率上限
#include "supercap.h"

extern float Kp = 10;
extern float Kd = 0.6;
extern int start_flag;
void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

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

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}
int CNT = 0;
void MotorUpdateTask(void* pvParameters)
{
    // 初始化功率限制器
    static bool power_limit_init = false;
    static uint32_t last_power_feedback_seq = 0;
    static bool bypass_power_limit_last = false;
    if (!power_limit_init) {
        power_limiter.Init();
        power_limit_init = true;
    }

    while (1)
    {
        TickType_t xlastWakeTime = xTaskGetTickCount();

        // 1. 计算 PID (Ontimer 会更新 motor.current)
        for (auto& motor : can1_motor) motor.Ontimer(can1.data, can1.temp_data);
        for (auto& motor : can2_motor) motor.Ontimer(can2.data, can2.temp_data);

        // 云台电机
        DM_motorYaw.DMmotorOntimer(can2.DMmotor_data, can2.DMmotor_temp_data_yaw);
        DM_motorPitch.DMmotorOntimer(can1.DMmotor_data, can1.DMmotor_temp_data_pitch);

        // =================================================================
        // 2. 功率限制计算
        // =================================================================
      
		// Read the referee power limit. Fall back to 80W when the referee is offline.
		float ref_limit = (judgement.data.robot_status_t.chassis_power_limit > 10)
			? (float)judgement.data.robot_status_t.chassis_power_limit
			: 80.0f;
		
	

      const bool bypass_power_limit = (ctrl.mode == CONTROL::FOLLOW);

		// Supercap board already receives referee limit and buffer data.
		// Do not locally raise the chassis power limit again, otherwise the two
		// control loops stack and easily cause over-power on the referee side.
        if (!bypass_power_limit)
		{
			if (bypass_power_limit_last)
			{
				power_limiter.Init();
			}
			power_limiter.Process(ctrl.chassis_motor, ctrl.chassis.power_1, ref_limit, true);
          const auto model_debug = power_limiter.GetModelDebugData();
			ctrl.chassis.power_model_debug.sum_torque_power = model_debug.sum_torque_power;
			ctrl.chassis.power_model_debug.k1_sum_abs_w = model_debug.k1_sum_abs_w;
			ctrl.chassis.power_model_debug.k2_sum_current_sq = model_debug.k2_sum_current_sq;
			ctrl.chassis.power_model_debug.k3 = model_debug.k3;
			ctrl.chassis.power_model_debug.predicted_power_raw = model_debug.predicted_power_raw;
			ctrl.chassis.power_model_debug.predicted_power_with_safety = model_debug.predicted_power_with_safety;
			ctrl.chassis.power_model_debug.real_power_feedback = model_debug.real_power_feedback;
           ctrl.chassis.power_model_predicted = power_limiter.GetPredictedPower();
		}
       else
		{
			ctrl.chassis.power_model_predicted = 0.0f;
           ctrl.chassis.power_model_debug.sum_torque_power = 0.0f;
			ctrl.chassis.power_model_debug.k1_sum_abs_w = 0.0f;
			ctrl.chassis.power_model_debug.k2_sum_current_sq = 0.0f;
			ctrl.chassis.power_model_debug.k3 = 0.0f;
			ctrl.chassis.power_model_debug.predicted_power_raw = 0.0f;
			ctrl.chassis.power_model_debug.predicted_power_with_safety = 0.0f;
			ctrl.chassis.power_model_debug.real_power_feedback = ctrl.chassis.power_1;
		}
		bypass_power_limit_last = bypass_power_limit;

        // =================================================================
        // 3. 【关键修正】将限制后的电流安全回填到 CAN2 缓冲区
        // =================================================================

        // 我们直接遍历 ctrl.chassis_motor，确保只处理底盘电机
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
            Motor* m = ctrl.chassis_motor[i];
            if (m == nullptr) continue;

            // 临时变量存储电流，方便移位操作
            int16_t safe_current = m->current;

            // 目标缓冲区指针
            uint8_t* pBuffer = nullptr;
            int buffer_index = -1;

            // -------------------------------------------------------------
            // 绝对 ID 映射 (Hardcoded Mapping) - 专治数据错位
            // -------------------------------------------------------------
            // 根据大疆电调协议：
            // ID 1 (0x201) -> 0x200帧 的 Byte 0, 1
            // ID 2 (0x202) -> 0x200帧 的 Byte 2, 3
            // ID 3 (0x203) -> 0x200帧 的 Byte 4, 5
            // ID 4 (0x204) -> 0x200帧 的 Byte 6, 7

            // 既然你说电机在 CAN2，我们只操作 can2.temp_data
            // 注意：CanTransimtTask 里发送 0x200 用的是 can2.temp_data

            if (m->ID == 0x205) {
                pBuffer = can2.temp_data;
                buffer_index = 8; // Offset 8
            }
            else if (m->ID == 0x206) {
                pBuffer = can2.temp_data;
                buffer_index = 10;
            }
            else if (m->ID == 0x207) {
                pBuffer = can2.temp_data;
                buffer_index = 12;
            }
            else if (m->ID == 0x208) {
                pBuffer = can2.temp_data;
                buffer_index = 14;
            }

            // 只有匹配到了才写入，防止数组越界或写错位置
            if (pBuffer != nullptr && buffer_index >= 0) {
                pBuffer[buffer_index] = (safe_current >> 8) & 0xFF; // 高8位
                pBuffer[buffer_index + 1] = safe_current & 0xFF;        // 低8位
            }
        }

        vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));
    }
}

void CanTransimtTask(void* pvParameters)
{
	while (true)
	{

		TickType_t xlastWakeTime1 = xTaskGetTickCount();
#if SENTRY_PITCH_ZERO_MODE
		PitchZeroUpdate();
		GimbalDiagnosticUpdate();
		vTaskDelay(pdMS_TO_TICKS(5));
		continue;
#endif
#if SENTRY_CALIBRATION_READ_ONLY
		static bool disable_sent = false;
		static uint32_t last_disable = 0;
		const uint32_t now = HAL_GetTick();
		if (!disable_sent || now - last_disable >= 500) {
			const uint8_t disable[8] = {255,255,255,255,255,255,255,0xFD};
			can1.Transmit(0x109, disable, 8);
			disable_sent = true; last_disable = now;
		}
		// DaMiao status refresh: no enable, motion, parameter write or zero command.
		const uint8_t query[8] = {0x09, 0x00, 0xCC, 0, 0, 0, 0, 0};
		can1.Transmit(0x7FF, query, 8);
		vTaskDelay(pdMS_TO_TICKS(50));
		continue;
#endif

		switch ((timer.counter++) % 4)
		{
		case 0:
			DM_motorYaw.DMmotorTransmit(0x206);
			DM_motorPitch.DMmotorTransmit(0x109);
			if (timer.counter % 250 == 4)
			{
				DM_motorYaw.MotorStart(0x206);
				DM_motorPitch.MotorStart(0x109);
			}
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data);
			can2.Transmit(0x200, can2.temp_data);
			break;
		case 3:
			can1.Transmit(0x2ff, can1.temp_data + 8);
			can2.Transmit(0x2ff, can2.temp_data + 8);
			break;
		default:
			break;
		}
		
		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//开始执行该任务之后1ms再执行该任务

	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
#if SENTRY_PITCH_ZERO_MODE
		vTaskDelay(pdMS_TO_TICKS(5));
		continue; // Dedicated gimbal controller owns commands in calibration mode.
#endif
		ctrl.chassis.Update();
		ctrl.pantile.Update();
		ctrl.shooter.Update();
		ctrl.automation.stateUpdate();
		rc.Update();
		xuc.Encode();
		vTaskDelay(1);
	}
}


void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();
		xuc.Decode();
		imu_pantile.Decode();
		imu_gimbal.Decode();
       supercap.decode();
#if !SENTRY_PITCH_ZERO_MODE
        judgement.BuffData();
        judgement.GetData();
#endif
       supercap.encode();
#if !SENTRY_PITCH_ZERO_MODE
        judgement.SendData();
#endif
		vTaskDelay(5);
	}
}







