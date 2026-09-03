#define private public
#include "xuc.h"
#undef private

#include <stdint.h>
#include <stdio.h>
#include <string.h>

int main()
{
	static_assert(sizeof(RxPacket_TJ) == 14, "host RxPacket_TJ size mismatch");
	static_assert(sizeof(TxPacket_TJ) == 20, "host TxPacket_TJ size mismatch");

	const uint8_t safe_payload[12] = {
		0x53, 0x50, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	if (XUC::CalculateCRC16(safe_payload, sizeof(safe_payload)) != 0xE637U)
	{
		return 1;
	}

	uint8_t safe_packet[14]{};
	memcpy(safe_packet, safe_payload, sizeof(safe_payload));
	XUC::AppendCRC16(safe_packet, sizeof(safe_packet));
	if (safe_packet[12] != 0x37U || safe_packet[13] != 0xE6U ||
		!XUC::VerifyCRC16(safe_packet, sizeof(safe_packet)))
	{
		return 2;
	}

	const uint8_t feedback_payload[18] = {
		0x53, 0x50, 0x01, 0x03, 0x00, 0x00, 0x68, 0x41, 0x01,
		0x00, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00, 0x3F
	};
	if (XUC::CalculateCRC16(feedback_payload, sizeof(feedback_payload)) != 0xEC87U)
	{
		return 3;
	}

	RxPacket_TJ command{};
	command.head[0] = 'S';
	command.head[1] = 'P';
	command.control_TJ = 1U;
	command.shoot_TJ = 0U;
	command.yaw_TJ = 0.25f;
	command.pitch_TJ = -0.5f;
	uint8_t command_bytes[sizeof(command)]{};
	memcpy(command_bytes, &command, sizeof(command));
	XUC::AppendCRC16(command_bytes, sizeof(command_bytes));

	XUC parser;
	const uint8_t noise[] = {0x11U, 0x22U, 'S'};
	parser.AppendReceiveBytes(noise, sizeof(noise));
	parser.ParseReceiveStream();
	parser.AppendReceiveBytes(command_bytes, 3U);
	parser.ParseReceiveStream();
	parser.AppendReceiveBytes(command_bytes + 3U, sizeof(command_bytes) - 3U);
	parser.ParseReceiveStream();
	if (parser.ValidPackets() != 1U || !parser.HasFreshCommand(0U) ||
		parser.HasFreshCommand(XUC_TJ_COMMAND_TIMEOUT_MS + 1U) ||
		parser.LatestCommand().control_TJ != 1U ||
		parser.LatestCommand().yaw_TJ != 0.25f ||
		parser.LatestCommand().pitch_TJ != -0.5f)
	{
		return 4;
	}

	uint8_t corrupt[sizeof(command_bytes)]{};
	memcpy(corrupt, command_bytes, sizeof(corrupt));
	corrupt[sizeof(corrupt) - 1U] ^= 0x5AU;
	parser.AppendReceiveBytes(corrupt, sizeof(corrupt));
	parser.AppendReceiveBytes(command_bytes, sizeof(command_bytes));
	parser.ParseReceiveStream();
	if (parser.CrcErrors() != 1U || parser.ValidPackets() != 2U)
	{
		return 5;
	}

	RxPacket_TJ nonfinite = command;
	const uint32_t quiet_nan = 0x7FC00000U;
	memcpy(&nonfinite.yaw_TJ, &quiet_nan, sizeof(quiet_nan));
	uint8_t nonfinite_bytes[sizeof(nonfinite)]{};
	memcpy(nonfinite_bytes, &nonfinite, sizeof(nonfinite));
	XUC::AppendCRC16(nonfinite_bytes, sizeof(nonfinite_bytes));
	parser.AppendReceiveBytes(nonfinite_bytes, sizeof(nonfinite_bytes));
	parser.ParseReceiveStream();
	if (parser.FieldErrors() != 1U || parser.ValidPackets() != 2U)
	{
		return 6;
	}

	RxPacket_TJ out_of_range = command;
	out_of_range.yaw_TJ = 1000.0f;
	uint8_t out_of_range_bytes[sizeof(out_of_range)]{};
	memcpy(out_of_range_bytes, &out_of_range, sizeof(out_of_range));
	XUC::AppendCRC16(out_of_range_bytes, sizeof(out_of_range_bytes));
	parser.AppendReceiveBytes(out_of_range_bytes, sizeof(out_of_range_bytes));
	parser.ParseReceiveStream();
	if (parser.FieldErrors() != 2U || parser.ValidPackets() != 2U)
	{
		return 7;
	}

	printf("RX_PACKET_SIZE=%u\n", static_cast<unsigned>(sizeof(RxPacket_TJ)));
	printf("TX_PACKET_SIZE=%u\n", static_cast<unsigned>(sizeof(TxPacket_TJ)));
	printf("CPP_STREAM_PARSER_TEST=PASS\n");
	printf("XUC_V2_CPP_CRC_TEST=PASS\n");
	return 0;
}
