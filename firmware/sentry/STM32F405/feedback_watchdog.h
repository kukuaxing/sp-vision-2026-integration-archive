#pragma once
#include <cstdint>

// The clock is renewed only by a validated physical CAN receive callback.
// Consumers may decode a cached frame repeatedly, but cannot renew its lease.
class FeedbackWatchdog final {
public:
	static constexpr uint32_t timeout_ms = 100;
	void Received(uint32_t now_ms) { last_ms_ = now_ms; received_ = true; }
	bool Fresh(uint32_t now_ms) const {
		return received_ && uint32_t(now_ms - last_ms_) <= timeout_ms;
	}
private:
	volatile uint32_t last_ms_ = 0;
	volatile bool received_ = false;
};
