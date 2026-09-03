#include "hipnuc_crc.h"

#include <stdint.h>
#include <stdio.h>

namespace
{
bool Verify(const uint8_t* frame, uint16_t frame_length)
{
	if (frame_length < 6U)
	{
		return false;
	}
	const uint16_t payload_length = static_cast<uint16_t>(
		static_cast<uint16_t>(frame[2]) |
		(static_cast<uint16_t>(frame[3]) << 8U));
	if (static_cast<uint32_t>(payload_length) + 6U != frame_length)
	{
		return false;
	}
	const uint16_t received = static_cast<uint16_t>(
		static_cast<uint16_t>(frame[4]) |
		(static_cast<uint16_t>(frame[5]) << 8U));
	return hipnuc::CalculateFrameCRC16(frame, payload_length) == received;
}
}

int main()
{
	// Atomic SWD capture from the robot's CH010 on 2026-09-03.
	uint8_t live_frame[] = {
		0x5A, 0xA5, 0x4C, 0x00, 0x02, 0x9D, 0x91, 0x23,
		0x23, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x5E, 0xCA,
		0x52, 0x00, 0xE9, 0xE9, 0xC8, 0xBD, 0xAD, 0x7D,
		0x0D, 0x3D, 0xB0, 0xC4, 0x7F, 0x3F, 0xA5, 0xB7,
		0x62, 0x3D, 0xFC, 0x90, 0x09, 0xBF, 0x80, 0xD7,
		0x66, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCB, 0x39,
		0xB8, 0x40,
		0x27, 0x0F, 0xE7, 0x3F, 0x2C, 0x65, 0x0A, 0x41,
		0x5A, 0xA8, 0x7E, 0x3F, 0xD1, 0x4B, 0x43, 0x3C,
		0x63, 0xC1, 0x51, 0x3D, 0xE7, 0xB2, 0x9B, 0x3D
	};
	if (!Verify(live_frame, sizeof(live_frame)) ||
		hipnuc::CalculateFrameCRC16(live_frame, 76U) != 0x9D02U)
	{
		return 1;
	}

	// A payload-only CRC was the original bug and must not be accepted.
	if (hipnuc::UpdateCRC16(0U, live_frame + 6U, 76U) == 0x9D02U)
	{
		return 2;
	}

	live_frame[20] ^= 0x01U;
	if (Verify(live_frame, sizeof(live_frame)))
	{
		return 3;
	}

	printf("HIPNUC_LIVE_FRAME_CRC=0x%04X\n", 0x9D02U);
	printf("HIPNUC_CRC_TEST=PASS\n");
	return 0;
}
