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
#include "DMmotor.h"
#include "nuc.h"
#include "PowerLimiter.h" // 寮曞叆澶存枃浠?
#include "judgement.h"    // 璇诲彇瑁佸垽绯荤粺鍔熺巼涓婇檺
#include "supercap.h"

extern float Kp = 10;
extern float Kd = 0.6;
extern int start_flag;
void TASK::Init()
{
	//鍒涘缓寮€濮嬩换鍔?
	xTaskCreate((TaskFunction_t)start_task,            //浠诲姟鍑芥暟
		(const char*)"start_task",          //浠诲姟鍚嶇О
		(uint16_t)START_STK_SIZE,        //浠诲姟鍫嗘爤澶у皬
		(void*)NULL,                  //浼犻€掔粰浠诲姟鍑芥暟鐨勫弬鏁?
		(UBaseType_t)START_TASK_PRIO,       //浠诲姟浼樺厛绾?
		(TaskHandle_t*)&StartTask_Handler);   //浠诲姟鍙ユ焺
	vTaskStartScheduler();          //寮€鍚换鍔¤皟搴?
}

/*
寮€濮嬩换鍔′换鍔″嚱鏁?
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //杩涘叆涓寸晫鍖?
	//鍒涘缓浠诲姟

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

	vTaskDelete(StartTask_Handler); //鍒犻櫎寮€濮嬩换鍔?
	taskEXIT_CRITICAL();            //閫€鍑轰复鐣屽尯
}
int CNT = 0;
void MotorUpdateTask(void* pvParameters)
{
    // 鍒濆鍖栧姛鐜囬檺鍒跺櫒
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

        // 1. 璁＄畻 PID (Ontimer 浼氭洿鏂?motor.current)
        for (auto& motor : can1_motor) motor.Ontimer(can1.data, can1.temp_data);
        for (auto& motor : can2_motor) motor.Ontimer(can2.data, can2.temp_data);

        // 浜戝彴鐢垫満
        DM_motorYaw.DMmotorOntimer(can2.DMmotor_data, can2.DMmotor_temp_data_yaw);
        DM_motorPitch.DMmotorOntimer(can1.DMmotor_data, can1.DMmotor_temp_data_pitch);

        // =================================================================
        // 2. 鍔熺巼闄愬埗璁＄畻
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
        // 3. 銆愬叧閿慨姝ｃ€戝皢闄愬埗鍚庣殑鐢垫祦瀹夊叏鍥炲～鍒?CAN2 缂撳啿鍖?
        // =================================================================

        // 鎴戜滑鐩存帴閬嶅巻 ctrl.chassis_motor锛岀‘淇濆彧澶勭悊搴曠洏鐢垫満
        for (int i = 0; i < CHASSIS_MOTOR_NUM; i++) {
            Motor* m = ctrl.chassis_motor[i];
            if (m == nullptr) continue;

            // 涓存椂鍙橀噺瀛樺偍鐢垫祦锛屾柟渚跨Щ浣嶆搷浣?
            int16_t safe_current = m->current;

            // 鐩爣缂撳啿鍖烘寚閽?
            uint8_t* pBuffer = nullptr;
            int buffer_index = -1;

            // -------------------------------------------------------------
            // 缁濆 ID 鏄犲皠 (Hardcoded Mapping) - 涓撴不鏁版嵁閿欎綅
            // -------------------------------------------------------------
            // 鏍规嵁澶х枂鐢佃皟鍗忚锛?
            // ID 1 (0x201) -> 0x200甯?鐨?Byte 0, 1
            // ID 2 (0x202) -> 0x200甯?鐨?Byte 2, 3
            // ID 3 (0x203) -> 0x200甯?鐨?Byte 4, 5
            // ID 4 (0x204) -> 0x200甯?鐨?Byte 6, 7

            // 鏃㈢劧浣犺鐢垫満鍦?CAN2锛屾垜浠彧鎿嶄綔 can2.temp_data
            // 娉ㄦ剰锛欳anTransimtTask 閲屽彂閫?0x200 鐢ㄧ殑鏄?can2.temp_data

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

            // 鍙湁鍖归厤鍒颁簡鎵嶅啓鍏ワ紝闃叉鏁扮粍瓒婄晫鎴栧啓閿欎綅缃?
            if (pBuffer != nullptr && buffer_index >= 0) {
                pBuffer[buffer_index] = (safe_current >> 8) & 0xFF; // 楂?浣?
                pBuffer[buffer_index + 1] = safe_current & 0xFF;        // 浣?浣?
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

		switch ((timer.counter++) % 4)
		{
		case 0:
			DM_motorYaw.DMmotorTransmit(0x206);
			DM_motorPitch.DMmotorTransmit(0x109);
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

		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//寮€濮嬫墽琛岃浠诲姟涔嬪悗1ms鍐嶆墽琛岃浠诲姟

	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
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
		judgement.BuffData();
		judgement.GetData();
       supercap.encode();
		judgement.SendData();
		vTaskDelay(5);
	}
}
