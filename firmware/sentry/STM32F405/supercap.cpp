#include "supercap.h"
#include "imu.h"
#include "label.h"
#include "control.h"
#include "motor.h"
#include <judgement.h>


float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
};

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void SUPERCAP::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
	huart->Init(uart_base, baud).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;

	//Set_PE_Target();
}

void SUPERCAP::decode()
{
	pd_Rx = xQueueReceive(*queueHandler, rxData, NULL);

	if (Rxsuper.I == I_pre) { count += 1; }
	else { count = 0; }
	if (count > 200) { connect = false; }
	else { connect = true; }

	/*memcpy(supercap_union.supertcap_data, rxData, 20);*/
	I_pre = Rxsuper.I;

	Rxsuper.header1 = rxData[0];
	Rxsuper.header2 = rxData[1];
	if (rxData[0] == 0x4c)
	{
		if (rxData[1] == 0x0b)
		{
			Rxsuper.connect_flag = rxData[2];
			Rxsuper.cap_energy = u8_to_float(rxData + 4);
			Rxsuper.U = u8_to_float(rxData + 8);
			Rxsuper.I = u8_to_float(rxData + 12);
			
			/*Rxsuper.stop = (rxData[15] << 8) | (0x00ff & rxData[14]);
			Rxsuper.open_flag = rxData[16];*/
		}
	}
}

void SUPERCAP::encode()
{
	//纯发 要改东西在rc里改
	int packet_size = sizeof(Txsuper);
	Txsuper.count++;
	
	// 将数据包复制到发送缓冲区
	memcpy(tx_data, &Txsuper, packet_size);
	if (Txsuper.count == 100)
	{
		Txsuper.count = 0;
	}


	// 计算并附加 CRC16 校验和
	//appendCRC16CheckSum(tx_data, packet_size);
	
	// 发送数据
	m_uart->UARTTransmit(tx_data, packet_size);
}

/**
 * @brief  功率控制的主函数
 * @param  _RF_power:裁判系统功率
 * @param  _motor_power:电机实时功率
 * @param  _remain_energy:裁判系统剩余能量
 * @param  motor_out_raw:速度环输出值，也即电流值
 * @author kainan
 */
void SUPERCAP::Control(float _RF_power, float _motor_power, float _remain_energy)
{
	/* 更新功率数据 */
	Update(_RF_power, _motor_power, _remain_energy);

	/* 限幅值计算 */
	Calc_motorLimit();

	/* 充电功率计算 */
	//if (cap_charge_enable == __ENABLE)
	//	Calc_capChargePower();
	//else
	//	cap_charge_power = 0;
}

/**
 * @brief  得到电容充电的允许功率
 * @param  None
 * @retval cap_charge_power：电容充电的允许功率
 * @author kainan
 */
float SUPERCAP::Get_capChargePower(void)
{
	return cap_charge_power;
}

/**
 * @brief  得到电机输出限幅比例
 * @param  None
 * @retval lim_scale：限幅比例，每个电机输出应当乘以这个值
 * @author kainan
 */
float SUPERCAP::Get_limScale(void)
{
	return lim_scale;
}


/**
 * @brief  计算电机是否功率限制，并在内部得到限幅值
 * @param  motor_out_raw：电机速度环的输出值，即电流值
 * @retval 电机是否收到功率限制
 * @author kainan
 */

void SUPERCAP::Calc_motorLimit()
{
	int limit_power_total = 0;/* 最大总电流 */

	/* 底盘使用电容供电(英雄和步兵)，使用电路采样板的采样功率进行电机功率控制 */
	if (control_loop == REAL_POWER_LOOP)
	{
		/* motor_power_target * 10相当于叠加一个前馈量，PID会好调一些 */
		limit_power_total = motor_power_target * 10 + motorLimitController.Position(motor_power_target-motor_power,100.f);
	}
	/* 无电流采样板，底盘使用裁判系统供电(哨兵)，使用剩余能量进行电机功率控制 */
	else
	{
		/*limit_current_total = RF_power_target * 10 
			+ motorLimitController.Position(remain_energy_target- remain_energy,100.f);*/
		limit_power_total = motorLimitController.Position(remain_energy_target - remain_energy, 100.f);
	}

	/* 如果不加限幅，当超功率时，limitation负大进行功率限制，scale也负大 */
	limit_power_total = _PowerCtrl_Constrain(limit_power_total, 0, max_power_out);

	float scale = 1.0f;
	float current_sum = 0.0f;
	float motor_current_max = 0.0f;

	/* 输出总电流 */
	for (uint8_t i = 0; i < motor_num; i++)
		current_sum += abs(ctrl.chassis_motor[i]->setcurrent) * 20.f / 16384.f;
	power_sum_total = current_sum * 24.f;

	/* 总电流超过限制值 */
	if (power_sum_total > limit_power_total)
	{
		scale = limit_power_total / power_sum_total;
	}
	else {
		scale = 1.0f;
	}

	lim_scale = scale;
}


/**
 * @brief  计算允许电容充电功率
 * @param  None
 * @retval None
 * @author kainan
 */
void SUPERCAP::Calc_capChargePower(void)
{
	cap_charge_power = RF_power_target - capChargeController.Position(remain_energy_target - remain_energy, 100.f);
	if (cap_charge_power < 0)
		cap_charge_power = 0;
	else {}
}

/**
 * @brief  更新功率数据的值
 * @param  电池功率，电机功率，RF剩余能量
 * @retval None
 * @author kainan
 */
void SUPERCAP::Update(float _RF_power, float _motor_power, float _remain_energy)
{
	static float last_remian_energy = 0;
	RF_power = _RF_power;
	motor_power = _motor_power;

	/* 裁判系统的数据是否更新 */
	if (_remain_energy != last_remian_energy)
	{
		remain_energy = _remain_energy;
		last_remian_energy = remain_energy;
	}
	else
		remain_energy += (RF_power_target - RF_power) * ctrl_period / 1000.0f;
	if (remain_energy >= 60.0f)
		remain_energy = 60.0f;

	/* 使用电机电流预测功率值 */
	
	Pin = Ct * Icmd * w + k1 * w * w + k2 * Icmd * Icmd;
}

/**
 * @brief  设置裁判系统功率，电机功率，剩余能量的目标值
 * @param  _RF_power_target:裁判系统功率目标值
 * @param  _motor_power_target:电机功率目标值
 * @param  _remain_energy_target:剩余能量目标值
 * @retval None
 * @author kainan
 */
void SUPERCAP::Set_PE_Target()
{
	motorLimitController.m_Kp = 10.f;
	motorLimitController.m_Ti = 0.f;
	motorLimitController.m_Td = 0.f;
}