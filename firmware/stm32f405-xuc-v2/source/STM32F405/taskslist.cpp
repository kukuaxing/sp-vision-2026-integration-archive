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
#include "Power_read.h"
#include "HTmotor.h"
#include "xuc.h"
#include "xuc_aim.h"
extern int start_flag;

namespace
{
bool XucArmSwitchSelected()
{
	// MID+DOWN was already a no-motion LOCK combination in the electrical baseline.
	return rc.rc.s[0] == RC::MID && rc.rc.s[1] == RC::DOWN;
}

bool RcSticksCentered()
{
	const int16_t deadband = 50;
	for (uint8_t index = 0U; index < 4U; index++)
	{
		if (rc.rc.ch[index] < -deadband || rc.rc.ch[index] > deadband)
		{
			return false;
		}
	}
	return true;
}

void HoldGimbalAtCurrentPosition()
{
	ctrl.pantile.target_imu_yaw = imu_pantile.GetAngleYaw();
	ctrl.pantile.manual_yaw_input = 0.0f;
	ctrl.pantile.manual_pitch_input = 0.0f;
	if (ctrl.pantile_motor[CONTROL::PANTILE::YAW] != nullptr)
	{
		ctrl.pantile.mark_yaw = ctrl.pantile_motor[CONTROL::PANTILE::YAW]->angle[now];
		ctrl.pantile_motor[CONTROL::PANTILE::YAW]->setangle = ctrl.pantile.mark_yaw;
		ctrl.pantile_motor[CONTROL::PANTILE::YAW]->setspeed = 0;
	}
	const float pitch_feedback = DMmotor[0].pos;
	if (pitch_feedback >= ctrl.pantile.pitch_min - 0.1f &&
		pitch_feedback <= ctrl.pantile.pitch_max + 0.1f)
	{
		DMmotor[0].setPos = std::max(
			std::min(pitch_feedback, ctrl.pantile.pitch_max), ctrl.pantile.pitch_min);
	}
}

void ForceShooterSafe()
{
	ctrl.shooter.openRub = false;
	ctrl.shooter.supply_bullet = false;
	ctrl.shooter.auto_shoot = false;
	ctrl.shooter.trigger_now = false;
	ctrl.shooter.trigger_rise = false;
}

void ApplyBoundedXucYawControl()
{
	const uint32_t now_ms = HAL_GetTick();
	const bool arm_switch = XucArmSwitchSelected();
	const RxPacket_TJ& command = xuc.LatestCommand();
	XucYawControlInput input{};
	input.now_ms = now_ms;
	input.rc_link_valid = rc.frame_valid &&
		static_cast<uint32_t>(now_ms - rc.last_frame_ms) < RC_LINK_TIMEOUT_MS;
	input.arm_switch_selected = arm_switch;
	input.sticks_centered = RcSticksCentered();
	// AUTO requires a recent frame whose CRC passed strictly. The legacy parser
	// may expose CRC-failed 0x91 frames for diagnostics, but they cannot arm XUC.
	input.imu_valid = imu_pantile.HasFreshStrictData(now_ms, 100U);
	input.command_fresh = xuc.HasFreshCommand(now_ms);
	input.command_control = command.control_TJ;
	input.current_yaw_deg = imu_pantile.GetStrictAngleYaw();
	// xuc_yaw_sign与xuc_imu_yaw_sign是两条独立配置：前者已经决定下发目标方向，
	// 不能再次套用IMU显示符号。实车日志确认协议yaw与CH010控制yaw同向。
	input.command_yaw_rad =
		XucYawController::ProtocolYawToLowerImuYawRadians(command.yaw_TJ);

	const XucYawControlOutput output = xuc_yaw_controller.Update(input);
	XucPitchControlInput pitch_input{};
	pitch_input.now_ms = now_ms;
	pitch_input.motion_authorized = output.armed;
	pitch_input.arm_switch_selected = arm_switch;
	pitch_input.command_control = command.control_TJ;
	pitch_input.current_pitch_deg = imu_pantile.GetStrictAnglePitch();
	pitch_input.current_motor_pos_rad = DMmotor[0].pos;
	pitch_input.command_pitch_rad = command.pitch_TJ;
	const XucPitchControlOutput pitch_output = xuc_pitch_controller.Update(pitch_input);

	if (output.armed && pitch_output.active)
	{
		ctrl.mode = CONTROL::AUTO;
		ctrl.Control_Chassis(0, 0, 0);
		ctrl.Control_Pantile(0, 0);
		ForceShooterSafe();
		ctrl.pantile.target_imu_yaw = output.target_yaw_deg;
		ctrl.pantile.target_imu_pitch = pitch_output.target_pitch_deg;
		DMmotor[0].setPos = pitch_output.target_motor_pos_rad;
	}
	else if (arm_switch)
	{
		// AUTO is selected directly by switch position, but an unmet motion gate
		// still keeps all actuators stopped until the controller can arm in place.
		ctrl.mode = CONTROL::LOCK;
		ctrl.Control_Chassis(0, 0, 0);
		ForceShooterSafe();
		HoldGimbalAtCurrentPosition();
	}
}
}
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

	xTaskCreate((TaskFunction_t)XucTask,
		(const char*)"XucTask",
		(uint16_t)XUC_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)XUC_TASK_PRIO,
		(TaskHandle_t*)&XucTask_Handler);

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}
int CNT = 0;
// ===== [后续修改开始] 电机反馈/输出更新：只保留DJI 3508/6020链路 =====
void MotorUpdateTask(void* pvParameters)
{
	
	while (1)
	{
		TickType_t xlastWakeTime = xTaskGetTickCount();
	
		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);

		for (auto& motor : can2_motor)motor.Ontimer(can2.data, can2.temp_data);

		vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));//开始执行该任务之后1ms再执行该任务
	}
}

// ===== [后续修改结束] 电机反馈/输出更新 =====
// ===== [后续修改开始] CAN发送任务：向CAN1底盘和CAN2 yaw发送DJI电机电流帧 =====
void CanTransimtTask(void* pvParameters)
{
	while (true)
	{

		TickType_t xlastWakeTime1 = xTaskGetTickCount();

		// ===== [后续修改开始] 达妙pitch电机：解码反馈→算控制帧→发送（1ms周期，挂CAN1） =====
		DMmotor[0].State_Decode(can1, can1.jointidata); // 解析CAN1达妙电机反馈到 pos/curSpeed/current
		DMmotor[0].DMmotor_Ontimer(can1, DMmotor[0].Kp, DMmotor[0].Kd, can1.jointpdata[DMmotor[0].ID - 1]); // 按setPos/setSpeed生成控制帧，写入ID对应缓冲区
		DMmotor[0].DMmotor_transmit(DM_ID3); // 发送达妙pitch电机控制帧（ID 0x103）
		// ===== [后续修改结束] 达妙pitch电机 =====

		switch ((timer.counter++) % 3)
		{
		case 0:
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data);
			can2.Transmit(0x200, can2.temp_data);
			break;
		default:
			break;
		}
		
		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//开始执行该任务之后1ms再执行该任务

	}
}

// ===== [后续修改结束] CAN发送任务 =====
// ===== [后续修改开始] 控制任务顺序：先读遥控器，再更新云台和底盘 =====
void ControlTask(void* pvParameters)
{
	while (true)
	{
		xuc.PollReceive();
		rc.Update();
		ApplyBoundedXucYawControl();
		ctrl.pantile.Update();
		ctrl.chassis.Update();
		ctrl.shooter.Update();
		vTaskDelay(5);
	}
}


// ===== [后续修改结束] 控制任务顺序 =====
// ===== [后续修改开始] 解码任务：按宏开关启用云台IMU解析 =====
void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();

#if PANTILE_IMU_ENABLE
		imu_pantile.Decode();
#endif
	
		vTaskDelay(5);
	}
}

// ===== [后续修改结束] 解码任务 =====
void ArmTask(void* pvParameters)
{
	while (true)
	{
		power.Send();
		vTaskDelay(100);
	}
}

void XucTask(void* pvParameters)
{
	(void)pvParameters;
	TickType_t last_wake_time = xTaskGetTickCount();
	const float degree_to_radian = PI / 180.0f;

	while (true)
	{
		float imu_pitch_rad = 0.0f;
		float imu_yaw_rad = 0.0f;
#if PANTILE_IMU_ENABLE
		if (imu_pantile.HasFreshStrictData(HAL_GetTick(), 100U))
		{
			// CH010 angles are consumed as degrees by the existing control loop.
			imu_pitch_rad = imu_pantile.GetStrictAnglePitch() * degree_to_radian;
			imu_yaw_rad = imu_pantile.GetStrictAngleYaw() * degree_to_radian;
		}
#endif

		// Report the operator-selected mode directly. Motion authorization remains
		// independently guarded by XucYawController and is never implied by mode=1.
		const uint8_t feedback_mode = XucArmSwitchSelected() ? 1U : 0U;
		// Phase 2D.5 diagnostics: shooting is disabled, so the otherwise-unused
		// bullet fields report yaw motor speed and the latched AUTO reason without
		// changing the packet layout.
		const float yaw_motor_speed_rpm =
			(ctrl.pantile_motor[CONTROL::PANTILE::YAW] != nullptr)
				? static_cast<float>(ctrl.pantile_motor[CONTROL::PANTILE::YAW]->curspeed)
				: 0.0f;
		const uint16_t pitch_reason = xuc_pitch_controller.LastDisarmReasonCode();
		const uint16_t combined_reason =
			(pitch_reason != XUC_PITCH_DISARM_NONE)
				? pitch_reason : xuc_yaw_controller.LastDisarmReasonCode();
		xuc.SendFeedback(feedback_mode, 0U, yaw_motor_speed_rpm,
			combined_reason, imu_pitch_rad, imu_yaw_rad);
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
	}
}
