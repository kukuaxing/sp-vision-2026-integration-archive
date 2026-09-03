#pragma once

#include <stdint.h>

namespace hipnuc
{
inline uint16_t UpdateCRC16(
	uint16_t current_crc, const uint8_t* data, uint16_t length)
{
	uint32_t crc = current_crc;
	for (uint16_t j = 0; j < length; ++j)
	{
		crc ^= static_cast<uint32_t>(data[j]) << 8U;
		for (uint8_t i = 0; i < 8U; ++i)
		{
			uint32_t next = crc << 1U;
			if ((crc & 0x8000U) != 0U)
			{
				next ^= 0x1021U;
			}
			crc = next;
		}
	}
	return static_cast<uint16_t>(crc & 0xFFFFU);
}

// HiPNUC binary frames cover SOF (5A A5), LEN and payload. The two CRC
// bytes at offsets 4 and 5 are deliberately skipped.
inline uint16_t CalculateFrameCRC16(
	const uint8_t* frame, uint16_t payload_length)
{
	uint16_t crc = UpdateCRC16(0U, frame, 4U);
	return UpdateCRC16(crc, frame + 6U, payload_length);
}
}
