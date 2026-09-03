#pragma once

#include "stm32f4xx_hal.h"
#include "usart.h"

#include <stdint.h>

// Small-computer link. This test branch uses USART6 (PC6 TX / PC7 RX) at
// 460800 baud. These constants intentionally live next to the protocol.
static const uint32_t XUC_TJ_BAUD_RATE = 460800U;
static const uint32_t XUC_TJ_COMMAND_TIMEOUT_MS = 200U;

#pragma pack(push, 1)
struct TxPacket_TJ  // lower board -> small computer
{
	uint8_t head[2];
	uint8_t mode_TJ;            // 0 idle, 1 auto aim, 2 small buff, 3 big buff
	uint8_t robot_id;           // 3 red, 103 blue, 0 unknown
	float bullet_speed_TJ;
	uint16_t bullet_count_TJ;
	float imu_pitch_TJ;         // rad
	float imu_yaw_TJ;           // rad
	uint16_t crc16_TJ;
};

struct RxPacket_TJ  // small computer -> lower board
{
	uint8_t head[2];
	uint8_t control_TJ;
	uint8_t shoot_TJ;
	float yaw_TJ;               // rad; Phase 2A uses this only through the bounded yaw gate
	float pitch_TJ;             // rad; deliberately ignored by Phase 2A
	uint16_t crc16_TJ;
};
#pragma pack(pop)

static_assert(sizeof(TxPacket_TJ) == 20, "TxPacket_TJ must be 20 bytes");
static_assert(sizeof(RxPacket_TJ) == 14, "RxPacket_TJ must be 14 bytes");

class XUC
{
public:
	void Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate);
	void PollReceive();

	void SendFeedback(
		uint8_t mode,
		uint8_t robot_id,
		float bullet_speed,
		uint16_t bullet_count,
		float imu_pitch_rad,
		float imu_yaw_rad);

	bool HasFreshCommand(uint32_t now_ms) const;
	const RxPacket_TJ& LatestCommand() const;

	uint32_t RxBytes() const { return rx_bytes_; }
	uint32_t HeaderCandidates() const { return header_candidates_; }
	uint32_t CrcErrors() const { return crc_errors_; }
	uint32_t FieldErrors() const { return field_errors_; }
	uint32_t ValidPackets() const { return valid_packets_; }
	uint32_t TxPackets() const { return tx_packets_; }
	uint32_t LastValidRxMs() const { return last_valid_rx_ms_; }

	static uint16_t CalculateCRC16(
		const uint8_t* data,
		uint32_t length,
		uint16_t initial = 0xFFFFU);
	static bool VerifyCRC16(const uint8_t* packet, uint32_t packet_size);
	static void AppendCRC16(uint8_t* packet, uint32_t packet_size);

private:
	static const uint16_t RX_STREAM_CAPACITY = 256U;

	void AppendReceiveBytes(const uint8_t* data, uint16_t length);
	void ParseReceiveStream();
	void ConsumeReceiveBytes(uint16_t count);
	static bool IsFiniteFloat(float value);
	static bool CommandFieldsValid(const RxPacket_TJ& packet);

	UART* uart_ = nullptr;
	RxPacket_TJ latest_command_{};
	bool have_valid_command_ = false;
	uint32_t last_valid_rx_ms_ = 0U;

	uint8_t uart_chunk_[UART_MAX_LEN]{};
	uint8_t receive_stream_[RX_STREAM_CAPACITY]{};
	uint16_t receive_stream_size_ = 0U;
	uint8_t transmit_buffer_[sizeof(TxPacket_TJ)]{};

	uint32_t rx_bytes_ = 0U;
	uint32_t header_candidates_ = 0U;
	uint32_t crc_errors_ = 0U;
	uint32_t field_errors_ = 0U;
	uint32_t valid_packets_ = 0U;
	uint32_t tx_packets_ = 0U;
};

extern XUC xuc;
