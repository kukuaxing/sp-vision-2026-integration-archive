#pragma once
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "can.h"
#include "queue.h"
#include "pid.h"
#include "string.h"

enum E_ENABLE {
	__DISABLE = 0,
	__ENABLE = 1
};

enum E_LOOP {
	REAL_POWER_LOOP = 0,		/* !底盘!有电流采样板(英雄和步兵)，使用电流采样板的采样功率进行电机功率控制 */
	REMAIN_ENERGY_LOOP = 1	/* !底盘!无电容与电流采样板(哨兵)，使用裁判系统供电，使用剩余能量进行电机功率控制 */
};

typedef enum {
	WORKING = 0, //正常工作
	DISCHARGE, //检录时放电
	SHUT, //底盘断电时关闭PWM
} working_state; //工作状态

struct RxSurPacket
{
	uint8_t header1;
	uint8_t header2;
	uint8_t connect_flag;
	float cap_energy;
	float U;
	float I;
	
};

struct TxSurPacket
{
	uint8_t header1 = 0x4A; //帧头，0x4A 1
	uint8_t header2 = 0x4B; //帧头，0x4B 1
	uint16_t limit; //功率上限 2
	working_state state; //工作状态 
	uint8_t buffer; //缓冲能量
	uint8_t count;
};

class SUPERCAP
{
public:
	/*union
	{
		uint8_t supertcap_data[20];
		struct
		{
			uint8_t header1;
			uint8_t header2;
			float buffer;
			float U;
			float I;
			uint16_t stop;
			uint8_t open_flag;
		}supercap;

	}supercap_union;*/

	working_state s = WORKING;
	TxSurPacket Txsuper;
	RxSurPacket Rxsuper;
	
	float I_pre;
	int count;
	bool connect = false;
	void Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base);
	void encode();
	void decode();

	int update_cnt = 0;

	/*<! 功率相关变量 为了方便调试 放在public */
	float RF_power;
	float motor_power;
	float remain_energy;
	float motor_power_target;
	float RF_power_target = 60.f;
	float remain_energy_target = 0.f;
	//以下是缝合的西交利物浦的变量名
	float Pin, w, Icmd, Ct = 1.99688994e-6f, k1, k2;

	template<typename Type>
	Type _PowerCtrl_Constrain(Type input, Type min, Type max) {
		if (input <= min)
			return min;
		else if (input >= max)
			return max;
		else return input;
	}

	/*void Load_capChargeController(float(*pFunc)(const float current, const float target));
	void Load_motorLimitController(float(*pFunc)(const float current, const float target));*/

	/* 主函数 */
	void Control(float _RF_power, float _motor_power, float _remain_energy);

	/* 设定目标参数 */
	void Set_PE_Target();

	/* 获得电容充电供电 */
	float Get_capChargePower(void);

	/* 获得功率控制值 */
	float Get_limScale(void);

	uint8_t motor_num=4; 					/*<! 底盘电机数量，舵轮底盘 = 8，麦轮底盘 = 4  */
	E_ENABLE cap_charge_enable; /*<! 电容充电开关 */
	E_LOOP control_loop = REMAIN_ENERGY_LOOP; // 或 REAL_POWER_LOOP   /*<! 实时功率环/剩余能量环 */
	uint8_t ctrl_period=20;				/*<! 控制周期,功率预测用 单位ms */
	float power_sum_total = 0.0f;
	int max_current_out=16000;			  /*<! 最大电流输出值，建议设置和电调最大电流值一致，如C620为16384 */
	int max_power_out = 200; //最大功率输出值

	float cap_charge_power;		  /*<! 电容允许充电功率 */
	float lim_scale;						/*<! 限幅比例，每个电机输出应当乘以这个值 */

	//float(*capChargeController)(const float current, const float target);
	//float(*motorLimitController)(const float current, const float target);
	PID motorLimitController, capChargeController;

	void Update(float _RF_power, float _motor_power, float _remain_energy);
	void Calc_motorLimit();
	void Calc_capChargePower();

private:
	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler;
	uint8_t rxData[UART_MAX_LEN];
	uint8_t tx_data[UART_MAX_LEN];
	UART* m_uart;
	CAN* m_can;

	float u8_to_float(uint8_t* p) {
		float s{}; uint8_t ch[4];
		ch[0] = p[3]; ch[1] = p[2]; ch[2] = p[1]; ch[3] = p[0];

		memcpy(&s, p, 4); return s;
	}

};

extern SUPERCAP supercap;

