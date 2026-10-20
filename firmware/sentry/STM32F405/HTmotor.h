#pragma once
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <cmath>
#include"label.h"
#include "can.h"
#include "feedback_watchdog.h"

constexpr auto MAXHIGH = 0.35;//轮腿最高伸展高度，待测
constexpr auto MINHIGH = 0.133;
constexpr auto MAXROLL = 15;//轮腿ROLL范围限制
#define INIT_ANGLE_F 3.979350f//3.595378198//180+26 3.665191429
//#define INIT_ANGLE_B -0.802851442f//-0.48869218//-22° -0.3839724;-0.523598775598//-30°//1.25423324
#define INIT_ANGLE_B -0.884955f
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03
#define CMD_CLEAR_MODE      0x04
#define MOTOR_MODE			0x100
#define P_MIN -4*PI    // Radians
#define P_MAX 4*PI        
#define V_MIN -10    // Rad/s
#define V_MAX 10
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define C_MAX 40.f
#define C_MIN -40.f
#define T_MIN -18.0f    //力矩
#define T_MAX 18.0f
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))



#define KT 1.4f
enum { DM_ID1 = 0x01, DM_ID2, DM_ID3, DM_ID4, DM_ID5 };
enum  POSITION { L_F, L_B, R_F, R_B, F_B ,Pitch,Yaw};
enum  FUCTION_MODE { MIT, SPEED, P_S };
typedef enum {
	CAN_PACKET_SET_DUTY = 0, //占空比模式
	CAN_PACKET_SET_CURRENT, //电流环模式
	CAN_PACKET_SET_CURRENT_BRAKE, // 电流刹车模式
	CAN_PACKET_SET_RPM, // 转速模式
	CAN_PACKET_SET_POS, // 位置模式
	CAN_PACKET_SET_ORIGIN_HERE, //设置原点模式
	CAN_PACKET_SET_POS_SPD, //位置速度环模式
} CAN_PACKET_ID;

class DMMOTOR
{
public:
	uint32_t ID;
	FUCTION_MODE function;
	POSITION position;

	float angle[2]{}, setAngle{}, deltaAngle{};//认为是安装电机时，初始姿态的电机与连杆夹角；
	float pos{}, setPos{};//转子位置，也是反馈回来的角度
	float curSpeed, setSpeed;
	float current, setCurrent;
	float torque, setTorque;
	float Kp = 10.f;
	float Kd = 0.6f;

	bool decoded = false;   //是否解码成功，解码成功后才可以使用数据
	uint8_t status = 0;      // 0=未使能 1=已使能 8~E=故障
	enum SAFETY_FAULT : uint8_t { SAFETY_OK = 0, FEEDBACK_TIMEOUT = 1, DRIVER_DISABLED = 2, DRIVER_FAULT = 3 };
	// Readable in SWD without changing the V2 wire protocol. Only an MCU reset
	// clears a runtime safety latch; setting STOP alone cannot restart a motor.
	volatile bool safety_fault_latched = false;
	volatile uint8_t safety_fault_reason = SAFETY_OK;

	float uint_to_float(int x_int, float x_min, float x_max, int bits);//计算用函数
	int float_to_uint(float x, float x_min, float x_max, int bits);

	void  CanComm_ControlCmd(CAN& hcan, uint8_t cmd);//电机模式设置
	void  ZeroPosition(CAN& hcan, uint32_t id);//校准零位置

	void  Motor_Start(CAN& hcan, uint32_t id);
	void DMmotorinit(CAN& hcan);//初始化，使能所有电机

	//启动电机会设置电机模式并零位校准
	void  Motor_Stop(CAN& hcan, uint32_t id);//电机失力

	DMMOTOR& State_Decode(uint8_t odata[][8]);//解码并接收数据
	bool OnFeedbackFrame(const uint8_t* bytes); // CAN receive callback only.
	bool FeedbackFresh() const {
		return feedback_watchdog_.Fresh(HAL_GetTick()) && decoded_generation_ == feedback_generation_;
	}
	uint32_t FeedbackGeneration() const { return feedback_generation_; }
	// Startup requires FC issued in physical STOP/RESET, then a NEW real status=1
	// feedback. Once operational, any lost/disabled/faulted feedback latches OFF
	// until the MCU is restarted in STOP/RESET; cached goals can never re-enable it.
	bool DriveReady() const {
		return !safety_fault_latched && ever_operational_ && FeedbackFresh() && decoded && status == 1u;
	}
	bool FaultLatched() const { return safety_fault_latched; }
	void DMmotor_Ontimer(float f_kp, float f_kd, uint8_t* odata);//电流计算，不包括发送
	void DMmotor_transmit(CAN& hcan);//使能并发送控制数据

	void SetTorque(float settorque);
	float GetPosition();
	float GetSpeed();
	float GetTorque();

	//都是对单个电机进行控制，要有ID 

	DMMOTOR(const uint32_t ID, FUCTION_MODE function, POSITION position) :ID(ID), function(function), position(position) {};//定义接口

private:
	FeedbackWatchdog feedback_watchdog_;
	uint8_t feedback_bytes_[8]{};
	volatile uint32_t feedback_generation_ = 0;
	uint32_t decoded_generation_ = 0;
	volatile uint32_t feedback_sequence_ = 0;
	volatile bool ever_operational_ = false;
	bool enable_requested_ = false;
	uint32_t enable_request_sequence_ = 0;
	bool command_encoded_ = false;
	uint32_t command_encoded_generation_ = 0;
	void LatchSafetyFault(uint8_t reason) {
		if (!safety_fault_latched) safety_fault_reason = reason;
		safety_fault_latched = true;
	}

};

extern DMMOTOR DMmotor[2];
