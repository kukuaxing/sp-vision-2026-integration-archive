#include "HTmotor.h"
#ifndef SENTINEL_ACTUATOR_HOST_TEST
#include "delay.h"
#endif
#include "can.h"
#include "control.h"
#include "stdio.h"

#define now 0
#define last 1
void buffer_append_int32(uint8_t* buffer, int32_t number, int16_t* index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}
void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t* index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

bool DMMOTOR::OnFeedbackFrame(const uint8_t* bytes)
{
	if (!bytes || (bytes[0] & 0x0fu) != (ID & 0x0fu)) return false;
	const auto state = bytes[0] >> 4;
	if (state != 0u && state != 1u && (state < 8u || state > 14u)) return false;
	const uint32_t now_ms = HAL_GetTick();
	const uint8_t previous_state = feedback_bytes_[0] >> 4;
	if (ever_operational_ && !feedback_watchdog_.Fresh(now_ms)) LatchSafetyFault(FEEDBACK_TIMEOUT);
	if (state > 1u) LatchSafetyFault(DRIVER_FAULT);
	else if (ever_operational_ && state == 0u) LatchSafetyFault(DRIVER_DISABLED);
	if (!feedback_watchdog_.Fresh(now_ms) || (previous_state > 1u && state <= 1u))
		++feedback_generation_;
	memcpy(feedback_bytes_, bytes, sizeof(feedback_bytes_));
	feedback_watchdog_.Received(now_ms);
	++feedback_sequence_;
	return true;
}
DMMOTOR& DMMOTOR::State_Decode(uint8_t idata[][8])//接收反馈数据
{
	(void)idata;
	uint8_t frame[8];
	taskENTER_CRITICAL();
	const bool fresh = feedback_watchdog_.Fresh(HAL_GetTick());
	const uint32_t generation = feedback_generation_;
	const uint32_t receive_sequence = feedback_sequence_;
	if (fresh) memcpy(frame, feedback_bytes_, sizeof(frame));
	taskEXIT_CRITICAL();
	if (!fresh) {
		if (ever_operational_) LatchSafetyFault(FEEDBACK_TIMEOUT);
		decoded = false; command_encoded_ = false; setSpeed = 0.f;
		return *this;
	}
	const bool recovering = !decoded || decoded_generation_ != generation;
	decoded_generation_ = generation;
	
	//浮点型数据
	//receive_data[0]=电机id
	uint8_t id = 0xFF;
	for (uint8_t i = 0; i < sizeof(DMmotor) / sizeof(DMmotor[0]); i++) {
		if (&DMmotor[i] == this) { id = i; break; }//获取当前电机的索引
	}
	if (id == 0xFF) return *this;

	const uint8_t s = frame[0];  //电机状态
	if ((s & 0x0F) != (this->ID & 0x0F)) return *this;   // 本槽不是我的帧 → 不改任何东西
	status = s >> 4;
	if (status > 1u) LatchSafetyFault(DRIVER_FAULT);
	else if (ever_operational_ && status == 0u) LatchSafetyFault(DRIVER_DISABLED);

	int direct = 0;
	int tmp_value = 0;
	tmp_value = (frame[1] << 8) | (frame[2]);//电机位置
	pos = uint_to_float(tmp_value, P_MIN, P_MAX, 16);//浮点型
	tmp_value = (frame[3] << 4) | (frame[4] >> 4);//转速
	curSpeed = uint_to_float(tmp_value, V_MIN, V_MAX, 12);//转浮点型
	tmp_value = (frame[5]) | ((frame[4] & 0x0f) << 8);
	current = uint_to_float(tmp_value, C_MIN, C_MAX, 12);
	torque = current * KT;//（力矩=电流*转矩常数，本产品转矩常数为 1.4Nm/A）
	decoded = true;
	if (recovering || !ever_operational_) {
		setPos = pos; setSpeed = 0.f; setTorque = 0.f; command_encoded_ = false;
	}
	if (!safety_fault_latched && enable_requested_ && receive_sequence != enable_request_sequence_ && status == 1u)
		ever_operational_ = true;
	
	return *this;
}


void DMMOTOR::DMmotor_transmit(CAN& hcan)
{
	if (ever_operational_ && (!FeedbackFresh() || !decoded)) LatchSafetyFault(FEEDBACK_TIMEOUT);
	else if (ever_operational_ && status == 0u) LatchSafetyFault(DRIVER_DISABLED);
	else if (status > 1u) LatchSafetyFault(DRIVER_FAULT);
	if (safety_fault_latched) {
		// Explicit latched stop. Periodic startup FC cannot undo a runtime fault.
		CanComm_ControlCmd(hcan, CMD_RESET_MODE);
		return;
	}
	// Before the first enabled feedback, wait: an FD here could race and undo
	// the startup FC before its first reply. Position packets do not prove enable.
	if (!DriveReady() || !command_encoded_ || command_encoded_generation_ != decoded_generation_) return;
	//can2.Transmit(this->ID +0x100, can2.jointpdata[this->ID - 1], 8);
	//这一段有点没看懂
	uint8_t slot = 0xFF;
	for (uint8_t i = 0; i < sizeof(DMmotor) / sizeof(DMmotor[0]); i++)
		if (&DMmotor[i] == this) { slot = i; break; }//获取当前电机的索引
	if (slot == 0xFF) return;

	uint32_t offset;//协议规定的控制帧 ID 偏移
	switch (function)
	{
	case MIT:   offset = 0x000; break;//MIT：控制帧 ID = CAN_ID
	case P_S:   offset = 0x100; break;//位置速度模式
	case SPEED: offset = 0x200; break;//速度模式
	default:    return;
	}
	hcan.Transmit(ID + offset, hcan.jointpdata[slot], 8);
}

void DMMOTOR::DMmotorinit(CAN& hcan)
{
	if (safety_fault_latched || ever_operational_ ||
	    (ctrl.mode != CONTROL::STOP && ctrl.mode != CONTROL::RESET)) return;
	if (!enable_requested_) {
		enable_request_sequence_ = feedback_sequence_;
		enable_requested_ = true;
	}
	CanComm_ControlCmd(hcan, CMD_MOTOR_MODE);
	delay.delay_ms(1);
}

void DMMOTOR::SetTorque(float settorque)
{
	setTorque = settorque;
}

float DMMOTOR::GetPosition()
{
	return angle[now];
}

float DMMOTOR::GetSpeed()
{
	return curSpeed;
}

float DMMOTOR::GetTorque()
{
	return torque;
}

void  DMMOTOR::CanComm_ControlCmd(CAN& hcan, uint8_t cmd)//使能帧
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00 };
	switch (cmd)
	{
	case CMD_MOTOR_MODE:
		buf[7] = 0xFC;//进入电机
		break;

	case CMD_RESET_MODE:
		buf[7] = 0xFD;//退出电机
		break;

	case CMD_ZERO_POSITION:
		buf[7] = 0xFE;//保存位置零点
		break;

	case CMD_CLEAR_MODE:
		buf[7] = 0xFB;//清除错误
		break;

	default:
		return; /* 直接退出函数 */
	}
	hcan.Transmit(this->ID, buf, 8);
}

float DMMOTOR::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int DMMOTOR::float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DMMOTOR::DMmotor_Ontimer(float f_kp, float f_kd, uint8_t* odata)
{
	uint32_t p = 0, v = 0, kp = 0, kd = 0, t = 0;//位置给定，速度给定，位置比例系数，位置微分系数，转矩给定值
	/* 限制输入的参数在定义的范围内 */
	LIMIT_MIN_MAX(setPos, P_MIN, P_MAX);
	LIMIT_MIN_MAX(setSpeed, V_MIN, V_MAX);
	LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
	LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
	LIMIT_MIN_MAX(setTorque, T_MIN, T_MAX);
	switch (this->function)
	{
	case MIT:
		// 根据协议，对float参数进行转换 
		p = float_to_uint(setPos, P_MIN, P_MAX, 16);//位置两个字节
		v = float_to_uint(setSpeed, V_MIN, V_MAX, 12);//速度12位
		kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);//比例系数12位
		kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);//速度系数12位
		t = float_to_uint(setTorque, T_MIN, T_MAX, 12);//前馈力矩（电流）
		//根据传输协议，把数据转换为CAN命令数据字段并存入输出缓冲区

		odata[0] = p >> 8;
		odata[1] = p & 0xFF;
		odata[2] = v >> 4;
		odata[3] = ((v & 0xF) << 4) | (kp >> 8);
		odata[4] = kp & 0xFF;
		odata[5] = kd >> 4;
		odata[6] = ((kd & 0xF) << 4) | (t >> 8);
		odata[7] = t & 0xff;
		break;
		//写得很好，但是无人在意(没有用到)

	case P_S:   //电机1：Pitch：float 位置 + float 速度
		memcpy(odata, &setPos, 4);
		memcpy(odata + 4, &setSpeed, 4);
		break;
		//注意这里，学长在这里蒙了，是因为MIT模式和P_S模式的区别
		//MIT模式是发送位置、速度、力矩、比例系数、微分系数，而P_S模式是发送位置和速度，所以在P_S模式下，只需要发送位置和速度即可。

	case SPEED: //电机2：Yaw：float 速度 + 4 字节 0
		memcpy(odata, &setSpeed, 4);
		memset(odata + 4, 0, 4);
		break;

	default:
		memset(odata, 0, 8);
		break;
	}
	command_encoded_generation_ = decoded_generation_;
	command_encoded_ = true;
}
