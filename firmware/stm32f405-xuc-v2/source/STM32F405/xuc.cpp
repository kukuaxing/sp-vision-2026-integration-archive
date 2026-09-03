#include "xuc.h"

#include <string.h>

namespace
{
const uint16_t CRC16_TABLE[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
}

XUC xuc;

void XUC::Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate)
{
	uart_ = huart;
	receive_stream_size_ = 0U;
	have_valid_command_ = false;
	last_valid_rx_ms_ = 0U;
	memset(&latest_command_, 0, sizeof(latest_command_));

	if (uart_ != nullptr)
	{
		uart_->Init(instance, baud_rate).DMARxInit();
	}
}

void XUC::PollReceive()
{
	if (uart_ == nullptr)
	{
		return;
	}

	while (xQueueReceive(uart_->UartQueueHandler, uart_chunk_, 0U) == pdTRUE)
	{
		uint32_t chunk_length = uart_->dataDmaNum;
		if (chunk_length > UART_MAX_LEN)
		{
			chunk_length = UART_MAX_LEN;
			field_errors_++;
		}

		if (chunk_length > 0U)
		{
			rx_bytes_ += chunk_length;
			AppendReceiveBytes(uart_chunk_, static_cast<uint16_t>(chunk_length));
			ParseReceiveStream();
		}
	}
}

void XUC::SendFeedback(
	uint8_t mode,
	uint8_t robot_id,
	float bullet_speed,
	uint16_t bullet_count,
	float imu_pitch_rad,
	float imu_yaw_rad)
{
	if (uart_ == nullptr)
	{
		return;
	}

	TxPacket_TJ packet{};
	packet.head[0] = 'S';
	packet.head[1] = 'P';
	packet.mode_TJ = (mode <= 3U) ? mode : 0U;
	packet.robot_id = robot_id;
	packet.bullet_speed_TJ = IsFiniteFloat(bullet_speed) ? bullet_speed : 0.0f;
	packet.bullet_count_TJ = bullet_count;
	packet.imu_pitch_TJ = IsFiniteFloat(imu_pitch_rad) ? imu_pitch_rad : 0.0f;
	packet.imu_yaw_TJ = IsFiniteFloat(imu_yaw_rad) ? imu_yaw_rad : 0.0f;
	packet.crc16_TJ = 0U;

	memcpy(transmit_buffer_, &packet, sizeof(packet));
	AppendCRC16(transmit_buffer_, sizeof(packet));
	uart_->UARTTransmit(transmit_buffer_, sizeof(packet));
	tx_packets_++;
}

bool XUC::HasFreshCommand(uint32_t now_ms) const
{
	return have_valid_command_ &&
		static_cast<uint32_t>(now_ms - last_valid_rx_ms_) <= XUC_TJ_COMMAND_TIMEOUT_MS;
}

const RxPacket_TJ& XUC::LatestCommand() const
{
	return latest_command_;
}

uint16_t XUC::CalculateCRC16(const uint8_t* data, uint32_t length, uint16_t initial)
{
	if (data == nullptr)
	{
		return 0xFFFFU;
	}

	uint16_t crc = initial;
	while (length-- > 0U)
	{
		const uint8_t table_index = static_cast<uint8_t>((crc ^ *data++) & 0x00FFU);
		crc = static_cast<uint16_t>((crc >> 8U) ^ CRC16_TABLE[table_index]);
	}
	return crc;
}

bool XUC::VerifyCRC16(const uint8_t* packet, uint32_t packet_size)
{
	if (packet == nullptr || packet_size <= 2U)
	{
		return false;
	}

	const uint16_t expected = CalculateCRC16(packet, packet_size - 2U);
	return packet[packet_size - 2U] == static_cast<uint8_t>(expected & 0x00FFU) &&
		packet[packet_size - 1U] == static_cast<uint8_t>((expected >> 8U) & 0x00FFU);
}

void XUC::AppendCRC16(uint8_t* packet, uint32_t packet_size)
{
	if (packet == nullptr || packet_size <= 2U)
	{
		return;
	}

	const uint16_t crc = CalculateCRC16(packet, packet_size - 2U);
	packet[packet_size - 2U] = static_cast<uint8_t>(crc & 0x00FFU);
	packet[packet_size - 1U] = static_cast<uint8_t>((crc >> 8U) & 0x00FFU);
}

void XUC::AppendReceiveBytes(const uint8_t* data, uint16_t length)
{
	if (data == nullptr || length == 0U)
	{
		return;
	}

	if (length >= RX_STREAM_CAPACITY)
	{
		data += length - RX_STREAM_CAPACITY;
		length = RX_STREAM_CAPACITY;
		receive_stream_size_ = 0U;
		field_errors_++;
	}

	const uint16_t required = static_cast<uint16_t>(receive_stream_size_ + length);
	if (required > RX_STREAM_CAPACITY)
	{
		const uint16_t discard = static_cast<uint16_t>(required - RX_STREAM_CAPACITY);
		ConsumeReceiveBytes(discard);
		field_errors_++;
	}

	memcpy(receive_stream_ + receive_stream_size_, data, length);
	receive_stream_size_ = static_cast<uint16_t>(receive_stream_size_ + length);
}

void XUC::ParseReceiveStream()
{
	const uint16_t packet_size = sizeof(RxPacket_TJ);

	while (receive_stream_size_ >= 2U)
	{
		uint16_t header_offset = 0U;
		while (header_offset + 1U < receive_stream_size_ &&
			!(receive_stream_[header_offset] == 'S' &&
				receive_stream_[header_offset + 1U] == 'P'))
		{
			header_offset++;
		}

		if (header_offset + 1U >= receive_stream_size_)
		{
			const bool keep_trailing_s = receive_stream_[receive_stream_size_ - 1U] == 'S';
			if (keep_trailing_s)
			{
				receive_stream_[0] = 'S';
				receive_stream_size_ = 1U;
			}
			else
			{
				receive_stream_size_ = 0U;
			}
			return;
		}

		if (header_offset > 0U)
		{
			ConsumeReceiveBytes(header_offset);
		}

		if (receive_stream_size_ < packet_size)
		{
			return;
		}

		header_candidates_++;
		if (!VerifyCRC16(receive_stream_, packet_size))
		{
			crc_errors_++;
			ConsumeReceiveBytes(1U);
			continue;
		}

		RxPacket_TJ candidate{};
		memcpy(&candidate, receive_stream_, packet_size);
		if (!CommandFieldsValid(candidate))
		{
			field_errors_++;
			ConsumeReceiveBytes(packet_size);
			continue;
		}

		latest_command_ = candidate;
		have_valid_command_ = true;
		last_valid_rx_ms_ = HAL_GetTick();
		valid_packets_++;
		ConsumeReceiveBytes(packet_size);
	}
}

void XUC::ConsumeReceiveBytes(uint16_t count)
{
	if (count >= receive_stream_size_)
	{
		receive_stream_size_ = 0U;
		return;
	}

	const uint16_t remaining = static_cast<uint16_t>(receive_stream_size_ - count);
	memmove(receive_stream_, receive_stream_ + count, remaining);
	receive_stream_size_ = remaining;
}

bool XUC::IsFiniteFloat(float value)
{
	uint32_t bits = 0U;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & 0x7F800000U) != 0x7F800000U;
}

bool XUC::CommandFieldsValid(const RxPacket_TJ& packet)
{
	const float angle_limit_rad = 6.283185307179586f;
	return packet.head[0] == 'S' &&
		packet.head[1] == 'P' &&
		packet.control_TJ <= 1U &&
		packet.shoot_TJ <= 1U &&
		IsFiniteFloat(packet.yaw_TJ) &&
		IsFiniteFloat(packet.pitch_TJ) &&
		packet.yaw_TJ >= -angle_limit_rad &&
		packet.yaw_TJ <= angle_limit_rad &&
		packet.pitch_TJ >= -angle_limit_rad &&
		packet.pitch_TJ <= angle_limit_rad;
}
