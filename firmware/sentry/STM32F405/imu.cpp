#include "imu.h"
#include "label.h"
#include "nuc.h"
#include "calibration_mode.h"
#include <cmath>

void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type)
{
	if (huart->FrameQueueHandler == NULL)
        huart->FrameQueueHandler = xQueueCreate(1, sizeof(UartRxFrame));
    configASSERT(huart->FrameQueueHandler != NULL);
    huart->Init(Instance, BaudRate).DMARxInit();
	m_uart = huart;
	this->type = type;
	queueHandler = &huart->FrameQueueHandler;
}

void IMU::Decode()
{
    if (queueHandler == NULL || *queueHandler == NULL) return;
    UartRxFrame packet{};
    pd_Rx = xQueueReceive(*queueHandler, &packet, 0);
    if (pd_Rx != pdTRUE) return; // Never reprocess the previous packet.
    if (packet.size > UART_MAX_LEN) { ++invalid_count; return; }
    memcpy(rxData, packet.data, packet.size);
    if (type == CH010 || type == HI226) {
        ImuPacket decoded{};
        if (!ParseImuPacket(packet.data, packet.size, decoded)) {
            ++invalid_count; return;
        }
        if (valid_count && decoded.sensor_time_ms == sensor_packet.sensor_time_ms) {
            ++repeated_count; return;
        }
        sensor_packet=decoded;
        acceleration.x=decoded.acceleration[0];
        acceleration.y=decoded.acceleration[1];
        acceleration.z=decoded.acceleration[2];
        angularvelocity.roll=decoded.gyro[0];
        angularvelocity.pitch=decoded.gyro[1];
        angularvelocity.yaw=decoded.gyro[2];
        angle.roll=decoded.rpy[0];
        angle.pitch=decoded.rpy[1];
        angle.yaw=decoded.rpy[2];
        last_valid_tick=HAL_GetTick();
        ++valid_count;
#if SENTRY_PITCH_ZERO_MODE
        if (this == &imu_pantile) xuc.ForwardCalibrationPose(packet.data, packet.size);
#endif
        return;
    }
    if (packet.size < 5 || size_t(rxData[3])+5 != packet.size) {
        ++invalid_count; return;
    }

	if (type == IMU601)
	{
        if ((rxData[2] == 0x01 && rxData[3] < 6) ||
            (rxData[2] == 0x03 && rxData[3] < 12)) { ++invalid_count; return; }
		if (rxData[0] == 0x55 && rxData[1] == 0x55)
		{
			uint8_t* data = rxData + 4;
			if (rxData[2] == 0x01 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				angle.roll = (float)getword(data[1], data[0]) * 180.f / 32768.f;
				angle.pitch = (float)getword(data[3], data[2]) * 180.f / 32768.f;
				angle.yaw = (float)getword(data[5], data[4]) * 180.f / 32768.f;
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

			}
		}
	}
}

float IMU::GetAngleYaw()
{
	return angle.yaw;
}

float IMU::getangularvelocitypitch()
{
	return angularvelocity.pitch;
}

float IMU::GetAnglePitch()
{
	return angle.pitch;
}

float IMU::GetAngleRoll()
{
	return angle.roll;
}

float* IMU::GetAcceleration()
{
	float temp[3]{};
	temp[0] = acceleration.x;
	temp[1] = acceleration.y;
	temp[2] = acceleration.z;
	return temp;
}
/*
void IMU::GetQ()
{
	double y = degreeToRadian(angle.yaw);
	double p = degreeToRadian(angle.pitch);
	double r = degreeToRadian(angle.roll);
	xuc.TxNuc_TJ.q_TJ[0] = cos(y / 2) * cos(p / 2) * cos(r/ 2) + sin(y / 2) * sin(p / 2) * sin(r/ 2);
	xuc.TxNuc_TJ.q_TJ[1] = sin(y/ 2) * cos(p/ 2) * cos(r / 2) - cos(y/ 2) * sin(p/ 2) * sin(r / 2);
	xuc.TxNuc_TJ.q_TJ[2] = cos(y / 2) * sin(p / 2) * cos(r/ 2) + sin(y / 2) * cos(p/ 2) * sin(r / 2);
	xuc.TxNuc_TJ.q_TJ[3] = cos(y/ 2) * cos(p / 2) * sin(r / 2) - sin(y / 2) * sin(p/ 2) * cos(r/ 2);
}
*/
bool IMU::Check(uint8_t* pdata, uint8_t len, uint32_t com)
{
	if (type == IMU601)
	{
		uint8_t t = 0;
		for (int i = 0; i < len; i++)
		{
			t += pdata[i];
		}
		return t == com;
	}
	return false;
}


int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)
{
	return HighBit << 8 | LowBits;
}