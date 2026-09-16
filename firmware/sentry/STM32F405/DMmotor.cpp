#include "DMmotor.h"
#include "delay.h"
#include "can.h"
#include "control.h"
#include "motor.h"
#include "kalman.h"
#include "label.h"
#include "LowPassFilter.h"
#include "nuc.h"

namespace {
	float NormalizeDeg180(float deg)
	{
		while (deg > 180.0f) deg -= 360.0f;
		while (deg <= -180.0f) deg += 360.0f;
		return deg;
	}
}

DMMOTOR::DMMOTOR(const uint32_t ID, CAN* hcan) //定义接口
	:ID(ID), mcan(hcan)
{}

void DMMOTOR::DMmotorTransmit(uint32_t id) //向电机发送数据包
{

	if (!initial)   //初始值为false
	{
		MotorStart(id);//电机使能
		initial = true;
		if (flag_first)
		{
			set_yaw_pos = initial_pos_yaw;
			set_pitch_pos = initial_pos_pitch;
			flag_first = false;
		}
	}
	else if (motorDisablity)
	{
		MotorOff(id);////电机失能
		motorDisablity = false;
	}
	else if (flag_zero)
	{
		MotorZero(id);//保存零点
		flag_zero = false;
	}
	else
	{
		if (id == 0x206)//yaw轴电机
		{
			mcan->Transmit(id, mcan->DMmotor_temp_data_yaw, 8);
		}
		if (id == 0x109)//pitch轴电机
		{
			mcan->Transmit(id, mcan->DMmotor_temp_data_pitch, 8);
		}
	}
}

void DMMOTOR::DMmotorOntimer(uint8_t idata[][8], uint8_t* odata)
{
	//接收数据并解码 浮点型数据
	uint8_t id;
	if (ID == 0x06)//大YAW
	{
		id = ID - 0x06;
		int direct = 0;
		int tmp_value = 0;
		tmp_value = (idata[id][1] << 8) | (idata[id][2]);//电机位置
		pos = uint_to_float(tmp_value, P_MIN, P_MAX, 16);//转浮点型
		pos_obs = pos * 10000;
		tmp_value = (idata[id][3] << 4) | (idata[id][4] >> 4);//转速
		cur_speed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//转浮点型
		temp_MOS = idata[id][6];//电机驱动温度
		temp_ROTOR = idata[id][7];//电机线圈温度
	}
	else if (ID == 0x09)//PITCH
	{
		id = ID - 0x08;
		int direct = 0;
		int tmp_value = 0;
		tmp_value = (idata[id][1] << 8) | (idata[id][2]);//电机位置
		pos = uint_to_float(tmp_value, P_MIN1, P_MAX1, 16);//转浮点型
		tmp_value = (idata[id][3] << 4) | (idata[id][4] >> 4);//转速
		cur_speed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//转浮点型
		temp_MOS = idata[id][6];//电机驱动温度
		temp_ROTOR = idata[id][7];//电机线圈温度

	}


	//打包数据准备发送
	if (ID == 0x06) //ID 0x06 大yaw轴电机 采用速度模式
	{
		// ========== 新方案：使用陀螺仪角度控制 ==========

		// 获取当前陀螺仪角度（度）
		float current_imu_yaw = imu_pantile.GetAngleYaw();
		const bool speed_control_mode =
			(ctrl.mode == CONTROL::ROTATION ||
				ctrl.mode == CONTROL::SHOOT ||
				ctrl.mode == CONTROL::RESET ||
				ctrl.mode == CONTROL::FOLLOW ||
				ctrl.chassis.transition_from_rotation ||
				ctrl.mode == CONTROL::NAVI);
		static bool yaw_cmd_initialized = false;
		static bool yaw_speed_mode_prev = true;
		static float yaw_cmd_filtered = 0.0f;

		if (!yaw_cmd_initialized)
		{
			yaw_cmd_filtered = current_imu_yaw;
			yaw_speed_mode_prev = speed_control_mode;
			yaw_cmd_initialized = true;
		}

		// 目标角度归一化到 [-180, 180]
		set_yaw_pos = NormalizeDeg180(set_yaw_pos);

		if (yaw_speed_mode_prev && !speed_control_mode)
		{
			// 速度环切回位置环首帧：锚定当前角，避免位置目标突变
			set_yaw_pos = current_imu_yaw;
			yaw_cmd_filtered = current_imu_yaw;
			DM_pid.ResetIntegral();
		}

		yaw_speed_mode_prev = speed_control_mode;

		float diff;
		if (speed_control_mode)
		{
			yaw_cmd_filtered = current_imu_yaw;
			diff = NormalizeDeg180(set_yaw_pos - current_imu_yaw);
		}
		else
		{
			// 位置环模式下对目标做限速，抑制模式切换/指令突变造成的冲击
			constexpr float YAW_SETPOINT_STEP_MAX = 0.2f; // deg/frame
			float cmd_delta = NormalizeDeg180(set_yaw_pos - yaw_cmd_filtered);
			if (cmd_delta > YAW_SETPOINT_STEP_MAX) cmd_delta = YAW_SETPOINT_STEP_MAX;
			if (cmd_delta < -YAW_SETPOINT_STEP_MAX) cmd_delta = -YAW_SETPOINT_STEP_MAX;
			yaw_cmd_filtered = NormalizeDeg180(yaw_cmd_filtered + cmd_delta);
			diff = NormalizeDeg180(yaw_cmd_filtered - current_imu_yaw);
		}

		// 转换为弧度供PID使用
		float diff_rad = diff * 0.017453292f;  // DEG_TO_RAD

		if (speed_control_mode)
		{
			// 旋转/RESET/FOLLOW/过渡期：直接使用 Keep_Pantile 设定的速度，不允许位置PID覆盖
			set_speed = setRange(set_speed, 3000.f);

		}
		else
		{
			// 正常模式：位置环控制（使用陀螺仪反馈）
			set_speed = DM_pid.Position(diff_rad, 100.f);
			set_speed = setRange(set_speed, 100.f);
		}

		uint8_t* vbuf;
		vbuf = (uint8_t*)&set_speed;

		odata[0] = *vbuf;
		odata[1] = *(vbuf + 1);
		odata[2] = *(vbuf + 2);
		odata[3] = *(vbuf + 3);
	}
	else if (ID == 0x09) //ID 0x09 pitch轴电机 采用位置速度模式 
	{
		if (ctrl.mode == CONTROL::AUTO || ctrl.mode == CONTROL::SHOOT || ctrl.mode == CONTROL::NAVI)
		{
			if (xuc.mode_TJ)
			{
				/*DM_motorPitch.set_pitch_pos = DM_motorPitch.pos + ctrl.pantile.pidAUTO[1].Position(xuc.pitch_TJ - degreeToRadian(imu_gimbal.angle.pitch), 500);*/
				DM_motorPitch.set_pitch_pos = DM_motorPitch.pos + (xuc.pitch_TJ - degreeToRadian(imu_gimbal.angle.pitch));

			}

			if (DM_motorPitch.set_pitch_pos > DM_motorPitch.DM_pitch_max)
			{
				DM_motorPitch.set_pitch_pos = DM_motorPitch.DM_pitch_max;
			}
			if (DM_motorPitch.set_pitch_pos < DM_motorPitch.DM_pitch_min)
			{
				DM_motorPitch.set_pitch_pos = DM_motorPitch.DM_pitch_min;
			}

		}

		uint8_t* pbuf1, * vbuf1;
		pbuf1 = (uint8_t*)&set_pitch_pos;
		vbuf1 = (uint8_t*)&set_pitch_speed;
		odata[0] = *pbuf1;
		odata[1] = *(pbuf1 + 1);
		odata[2] = *(pbuf1 + 2);
		odata[3] = *(pbuf1 + 3);
		odata[4] = *vbuf1;
		odata[5] = *(vbuf1 + 1);
		odata[6] = *(vbuf1 + 2);
		odata[7] = *(vbuf1 + 3);
	}
}

void DMMOTOR::MotorStart(uint32_t id) //电机使能
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
	mcan->Transmit(id, buf, 8);
}

void DMMOTOR::MotorOff(uint32_t id) //电机失能
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };
	mcan->Transmit(id, buf, 8);

}

void DMMOTOR::MotorZero(uint32_t id) //保存零点
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };
	mcan->Transmit(id, buf, 8);
}

void DMMOTOR::setDMmotor(float delta_pos, uint32_t id)
{
	if (id == DM_ID_Yaw)
	{
		DM_motorYaw.target_pos_yaw -= delta_pos;
	}

	if (id == DM_ID_Pitch)
	{
		DM_motorPitch.target_pos_pitch -= delta_pos;
	}
}
float DMMOTOR::setRange(const float original, const float range)
{
	return std::max(std::min(range, original), -range);
}