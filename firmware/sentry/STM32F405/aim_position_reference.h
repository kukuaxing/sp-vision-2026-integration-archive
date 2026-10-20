#pragma once
#include <algorithm>
#include <cmath>

// A reference is measured position + bounded correction, never old goal + error.
// Each takeover begins at measured position, including recovery from lost feedback.
class AimPositionReference final
{
public:
	void Reset() { anchored_ = false; }
	bool Update(float measured_position, float correction, float max_lead, float& reference)
	{
		if (!std::isfinite(measured_position) || !std::isfinite(correction) ||
		    !std::isfinite(max_lead) || max_lead <= 0.f) {
			Reset();
			return false;
		}
		reference = measured_position;
		if (anchored_) reference += std::max(-max_lead, std::min(max_lead, correction));
		anchored_ = true;
		return true;
	}

private:
	bool anchored_ = false;
};
