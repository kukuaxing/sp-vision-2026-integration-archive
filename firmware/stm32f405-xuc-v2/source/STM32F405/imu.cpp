#include "imu.h"
#include "hipnuc_crc.h"
#include "label.h"
void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type)
{
	huart->Init(Instance, BaudRate).DMARxInit();
	m_uart = huart;
	this->type = type;
	queueHandler = &huart->UartQueueHandler;
}

void IMU::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 鎴栬€呮姤閿?
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, rxData, 0);
	}
	if (pd_Rx != pdTRUE)
	{
		return;
	}

	parse_ok = 0;

	if (type == IMU601)
	{
		if (rxData[0] == 0x55 && rxData[1] == 0x55)
		{
			uint8_t* data = rxData + 4;
			if (rxData[2] == 0x01 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				angle.roll = (float)getword(data[1], data[0]) * 180.f / 32768.f;
				angle.pitch = (float)getword(data[3], data[2]) * 180.f / 32768.f;
				angle.yaw = (float)getword(data[5], data[4]) * 180.f / 32768.f;
				parse_ok = 1;
				decode_count++; // 成功解析计数递增
				last_decode_ms = HAL_GetTick();
				last_strict_decode_ms = last_decode_ms;
				strict_angle = angle;
			}
			else if (rxData[2] == 0x03 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				uint8_t Ax, Ay, Az, Gx, Gy, Gz;
				Ax = getword(data[1], data[0]);
				Ay = getword(data[3], data[2]);
				Az = getword(data[5], data[4]);
				Gx = getword(data[7], data[6]);
				Gy = getword(data[9], data[8]);
				Gz = getword(data[11], data[10]);
				acceleration.x = (float)Ax / 32768 * ACC_FSR;
				acceleration.y = (float)Ay / 32768 * ACC_FSR;
				acceleration.z = (float)Az / 32768 * ACC_FSR;
				angularvelocity.roll = (float)Gx / 32768 * GYRO_FSR;
				angularvelocity.pitch = (float)Gy / 32768 * GYRO_FSR;
				angularvelocity.yaw = (float)Gz / 32768 * GYRO_FSR;
				parse_ok = 1;
				decode_count++; // 成功解析计数递增
				last_decode_ms = HAL_GetTick();
				last_strict_decode_ms = last_decode_ms;
				strict_angle = angle;

			}
		}
	}
// ===== [后续修改开始] CH010/HI226帧扫描解析：支持DMA缓冲区错位查找 =====
	else if (type == CH010 || type == HI226)
	{
		uint16_t rx_len = UART_MAX_LEN; // 默认按最大DMA缓冲区长度扫描
		bool header_found = false; // 记录本次缓冲区是否找到CH010帧头
		if (m_uart != nullptr && m_uart->dataDmaNum > 0U && m_uart->dataDmaNum <= UART_MAX_LEN) // 若UART记录了实际DMA长度，则按实际长度扫描
		{
			rx_len = static_cast<uint16_t>(m_uart->dataDmaNum); // 使用实际接收长度减少无效字节干扰
		}

		for (uint16_t offset = 0; offset + 6U <= rx_len; offset++) // 在DMA缓冲区中逐字节寻找CH010帧头
		{
			if (rxData[offset] != 0x5A || rxData[offset + 1U] != 0xA5) // CH010帧头固定为0x5A 0xA5
			{
				continue;
			}

			header_found = true; // 找到候选帧头
			uint8_t* frame = rxData + offset; // 将frame指向当前候选帧起始位置
			const uint16_t data_len = ((uint16_t)frame[3] << 8) + frame[2]; // 读取CH010数据区长度，小端低字节在前
			const uint16_t frame_crc = ((uint16_t)frame[5] << 8) + frame[4]; // 读取帧内CRC字段
			const uint32_t total_len = (uint32_t)data_len + 6U; // 总帧长=数据区长度+6字节头部

			if (data_len < 60U || total_len > UART_MAX_LEN || offset + total_len > rx_len) // 长度异常或帧不完整则跳过
			{
				continue;
			}

			crc = 0;
			const bool crc_ok = Check(
				frame, static_cast<uint16_t>(total_len), frame_crc); // 校验帧头、长度和数据域
			crc = 0;
			if (!crc_ok)
			{
				crc_error_count++; // CRC异常计数，调试时可判断线序/波特率/协议问题
				if (frame[6] != 0x91) // 0x91为当前使用的姿态数据帧类型
				{
					continue;
				}
			}

			if (frame[6] == 0x91) // 姿态数据帧：包含加速度、角速度、欧拉角
			{
				int data_offset = 6; // 数据区从帧头后第6字节开始
				acceleration.x = R4(frame + data_offset + 12); // 解析x轴加速度float
				acceleration.y = R4(frame + data_offset + 16); // 解析y轴加速度float
				acceleration.z = R4(frame + data_offset + 20); // 解析z轴加速度float
				angularvelocity.pitch = R4(frame + data_offset + 24); // 解析pitch角速度float
				angularvelocity.roll = R4(frame + data_offset + 28); // 解析roll角速度float
				angularvelocity.yaw = R4(frame + data_offset + 32); // 解析yaw角速度float，供小陀螺前馈使用
				angle.pitch = R4(frame + data_offset + 48); // 解析pitch角度float
				angle.roll = R4(frame + data_offset + 52); // 解析roll角度float
				angle.yaw = R4(frame + data_offset + 56); // 解析yaw角度float，供云台锁角使用
				frame_offset = offset; // 记录有效帧偏移，便于Live Watch确认是否错位
				frame_len = static_cast<uint16_t>(total_len); // 记录有效帧长度
				parse_ok = crc_ok ? 1 : 2; // 1表示CRC正确，2表示CRC异常但按0x91帧解析
				decode_count++; // 成功解析计数递增
				last_decode_ms = HAL_GetTick();
				if (crc_ok)
				{
					last_strict_decode_ms = last_decode_ms;
					strict_angle = angle;
				}
				return;
			}
		}

		if (!header_found)
		{
			header_error_count++; // 本次缓冲区未找到帧头，累计帧头错误
		}
	}
}
// ===== [后续修改结束] CH010/HI226帧扫描解析 =====
float IMU::GetAngleYaw()
{
	return angle.yaw;
}

float IMU::GetStrictAngleYaw() const
{
	return strict_angle.yaw;
}

float IMU::getangularvelocitypitch()
{
	return angularvelocity.pitch;
}

// ===== [后续修改开始] 提供yaw角速度给小陀螺锁角前馈 =====
float IMU::GetAngularVelocityYaw() // 获取yaw角速度，供云台小陀螺锁角前馈调用
{
	return angularvelocity.yaw;
// ===== [后续修改结束] yaw角速度接口 =====
}

bool IMU::HasFreshData(uint32_t now_ms, uint32_t timeout_ms) const
{
	return decode_count > 0U &&
		static_cast<uint32_t>(now_ms - last_decode_ms) <= timeout_ms;
}

bool IMU::HasFreshStrictData(uint32_t now_ms, uint32_t timeout_ms) const
{
	return last_strict_decode_ms > 0U &&
		static_cast<uint32_t>(now_ms - last_strict_decode_ms) <= timeout_ms;
}

float IMU::GetAnglePitch()
{
	return angle.pitch;
}

float IMU::GetStrictAnglePitch() const
{
	return strict_angle.pitch;
}

float IMU::GetAngleRoll()
{
	return angle.roll;
}

IMU::Acceleration IMU::GetAcceleration()
{
	return acceleration;
}

bool IMU::Check(uint8_t* pdata, uint16_t len, uint32_t com)
{
	if (type == IMU601)
	{
		uint8_t t = 0;
		for (uint16_t i = 0; i < len; i++)
		{
			t += pdata[i];
		}
		return t == com;
	}
	else if (type == CH010 || type == HI226)
	{
		if (pdata == nullptr || len < 6U)
		{
			return false;
		}
		const uint16_t payload_len = static_cast<uint16_t>(
			static_cast<uint16_t>(pdata[2]) |
			(static_cast<uint16_t>(pdata[3]) << 8U));
		if (static_cast<uint32_t>(payload_len) + 6U != len)
		{
			return false;
		}
		crc_calc = hipnuc::CalculateFrameCRC16(pdata, payload_len);
		crc = crc_calc;
		crc_recv = static_cast<uint16_t>(com & 0xFFFFU); // 保存帧内接收到的CRC值
		return crc_calc == crc_recv;
	}
	return false;
}


int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)
{
	return HighBit << 8 | LowBits;
}
