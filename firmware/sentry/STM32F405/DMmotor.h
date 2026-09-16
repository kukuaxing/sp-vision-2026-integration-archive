#pragma once
#include <cinttypes>
#include <cstring>
#include <cmath>
#include"label.h"
#include "can.h"
#include "motor.h"
#include "kalman.h"

#define P_MIN -3.141593    // Radians
#define P_MAX 3.141593
#define P_MIN1 -12.56637    // Radians
#define P_MAX1 12.56637
#define V_MIN -45    // Rad/s
#define V_MAX 45
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define C_MAX 40.f		//电流
#define C_MIN -40.f
#define T_MIN -20.0f    //力矩
#define T_MAX 20.0f
#define KT 1.4f
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

enum ERR { overVoltage = 8, underVoltage, overCurrent, mosOverT, motorOverT, missConnect, overLoad };



class DMMOTOR
{
public:
	uint32_t ID;
	uint32_t DM_ID_Yaw = 0x06;
	uint32_t DM_ID_Pitch = 0x09;
	//function_type function;
	ERR error;
	DMMOTOR(const uint32_t ID,  CAN* hcan);//定义接口
	void DMmotorOntimer(uint8_t idata[][8], uint8_t* odata); //接收数据并解码 打包数据并发送
	void DMmotorTransmit(uint32_t id);//传输数据
	void MotorStart(uint32_t id);//电机使能
	void MotorOff(uint32_t id);//电机失能
	void MotorZero(uint32_t id);//保存电机零点
	void setDMmotor(float delta_pos, uint32_t id);//控制函数
	float setRange(const float original, const float range);

	float sensitivity = 0.04;
	bool initial = false;
	bool motorDisablity = false;
	bool flag_zero = false;
	bool flag_first = true;
	bool pos_initialized = false;  // 是否已收到第一帧有效CAN数据
	uint32_t last_rx_count_seen = 0U;
	uint32_t last_feedback_ms = 0U;
	uint32_t last_enable_attempt_ms = 0U;
	uint32_t first_feedback_ms = 0U;
	uint32_t last_pitch_command_ms = 0U;
	bool pitch_reference_valid = false;
	bool pitch_control_release_seen = false;
	bool pitch_motion_ready = false;
	PID DM_pid { 5.f,0.f,2.f,0.05f,1000.f};//DM电机 速度模式下 由位置到速度环PID
	PID DM_IMU_pid { 0.f,0.f,0.f,0.f,0.f };
	PID DM_IMU_ROTA_pid { 30.0f,0.0f,0.0f, 20.f, 20.f };
	PID DM_AUTO_pid{ 600.f,0.f,0.f,0.f,0.f };
	Kalman DM_kalman{ 1.f,40.f };

	float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
	}

	uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
	{
		float span = x_max - x_min;
		float offset = x_min;

		return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
	}


	CAN* mcan;
	float pos_obs;
	float angle[2]{};//认为是安装电机时，初始姿态的电机与连杆夹角；
	float pos;//转子位置，也是反馈回来的角度
	float cur_speed;//电机当前速度
	float temp_ROTOR, temp_MOS;//电机内部线圈温度 电机驱动上MOS温度
	float set_speed = 0.f;//给定速度
	float feedforward = 0.f;//前馈速度
	float startup_pos_yaw = 0.f;  // Yaw电机上电时实际位置(rad)，Keep_Direction参考基准
	float startup_pos_pitch = 0.f;// Pitch首次有效反馈位置，只能由本机反馈建立
	float pitch_command_sent = 0.f;
	float set_yaw_pos;//输出至电机的目标值 （处理后）
	float set_pitch_pos, set_pitch_speed = 6.66f;
	float target_pos_yaw, target_pos_pitch;
	float wheel_radius = 0.076f;      // m
	float chassis_radius = 0.25f;     // m
	float gear_ratio = 19.2f;
	float initial_pos_yaw = -0.16f, initial_pos_pitch = 0.0f;
	// Pitch绝对零位与机械限位必须在本车标定。启动后由首次有效反馈改写为临时软限位。
	float DM_pitch_min = -0.20f, DM_pitch_max = 0.20f;
	float rota_speed = 5.f, nomal_speed = 0.f;
	float rotationSpeed{};
	//float current, setCurrent;//电流
	//float torque, setTorque;//扭矩
private:
};

extern DMMOTOR DM_motorYaw;
extern DMMOTOR DM_motorPitch;
