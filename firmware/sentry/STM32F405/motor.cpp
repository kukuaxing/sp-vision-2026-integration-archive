#include "motor.h"
#include "HTmotor.h"
#define DEG_TO_RAD 0.017453292f  // π / 180
Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[speed2], &_speed2, sizeof(PID));
	this->function = function;
}


Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	this->function = function;
}

void Motor::StatusIdentifier(int32_t torque_current)
{
	if (torque_current == old_torque_current)
		disconnectCount++;
	else
		disconnectCount = 0;

	if (disconnectCount >= disconnectMax)
	{
		disconnectCount = disconnectMax;
		if (old_torque_current == 0)
			m_status = UNCONNECTED;
		else
			m_status = DISCONNECTED;
	}
	else
		m_status = FINE;

	old_torque_current = torque_current;
}
uint8_t Motor::getStatus()const
{
	return (uint8_t)m_status;
}
bool Motor::OnFeedbackFrame(const uint8_t* bytes)
{
	if (!bytes || ((uint16_t(bytes[0]) << 8) | bytes[1]) >= 8192u) return false;
	const uint32_t now_ms = HAL_GetTick();
	if (!feedback_watchdog_.Fresh(now_ms)) ++feedback_generation_;
	memcpy(feedback_bytes_, bytes, sizeof(feedback_bytes_));
	feedback_watchdog_.Received(now_ms);
	return true;
}
void Motor::CancelPositionTarget()
{
	taskENTER_CRITICAL();
	setangle = use_sum_angle ? float(sum_angle) : angle[now];
	setspeed = 0; current = 0;
	for (auto& controller : pid) controller.Reset();
	taskEXIT_CRITICAL();
}
void Motor::Ontimer(uint8_t idata[][8], uint8_t* odata)//idate: receive;odate: trainsmit;RC
{
	uint32_t trainsmit_or_receive_ID = this->ID - ID1;
	(void)idata; // A cached public CAN buffer is never evidence of a new arrival.
	uint8_t frame[8];
	taskENTER_CRITICAL();
	const bool fresh = feedback_watchdog_.Fresh(HAL_GetTick());
	const uint32_t generation = feedback_generation_;
	if (fresh) memcpy(frame, feedback_bytes_, sizeof(frame));
	// Keep controller state and the output pair coherent with cancellation in
	// ControlTask. No blocking operations occur in this bounded critical section.
	if (!fresh) {
		CancelPositionTarget();
		has_feedback = angle_latched = false;
		curspeed = 0; m_status = DISCONNECTED;
		odata[trainsmit_or_receive_ID * 2] = 0;
		odata[trainsmit_or_receive_ID * 2 + 1] = 0;
		taskEXIT_CRITICAL();
		return;
	}
	const bool recovering = !has_feedback || decoded_generation_ != generation;
	decoded_generation_ = generation;
	has_feedback = true; m_status = FINE;

	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID += 4;//神秘注释，好像是另外一套偏移id的方案
	}*/
	//----------------------------------------------------------------
	this->torque_current = getword(frame[4], frame[5]);
	this->angle[now] = getword(frame[0], frame[1]);
	this->temperature = frame[6];
	if (recovering) {
		angle[pre] = angle[now];
		CancelPositionTarget();
	}
	//Get currrent speed

	motor_status = 0;
	if (temperature > 70) {
		setspeed = 0;
	}

	if (type == EC60)
	{
		curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
	}
	else {
		curspeed = getword(frame[2], frame[3]);
	}
	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID -= 4;
	}*/
	//----------------------------------------------------------------
	// A wheel OFF/thermal stop is an output revocation, not a zero-speed
	// regulation request. The incremental speed loop can retain nonzero current
	// even at zero measured speed (observed on the stopped hot right wheel).
	// Clear both PID debt and the actual CAN pair; keep decoding encoder history.
	if (function == function_type::shooter && (setspeed == 0 || temperature > 70)) {
		CancelPositionTarget();
		recorded_the_Laps();
		GetDistanceFromMechanicalAngle();
		angle[pre] = angle[now];
		odata[trainsmit_or_receive_ID * 2] = 0;
		odata[trainsmit_or_receive_ID * 2 + 1] = 0;
		taskEXIT_CRITICAL();
		return;
	}
	//20220121--hz
	if (mode == ACE)
	{

	}
	else if (mode == POS)
	{
		if (!has_feedback) current = 0;
		else
		{
			if (!angle_latched)
			{
				if (use_sum_angle)                       // 小yaw和拨弹轮的POS模式不同，拨弹轮不能回环，要多圈
				{
					sum_angle = 0;
					angle[pre] = angle[now];             // ← 关键，见下
					setangle = 0.f;
				}
				else setangle = angle[now];              // 云台：单圈绝对角
				angle_latched = true;
			}

			float raw = use_sum_angle ? (setangle - (float)sum_angle)
				: (float)getdeltaa((int16_t)(setangle - angle[now]));//只有小yaw的POS模式才会用到getdeltaa，拨弹轮的POS模式不需要回环
			float pos_error = mechanicalToDegree(raw);

			if (fabsf(pos_error) < pos_deadband) pos_error = 0.f;// 小死区，防震荡

			float speed_ref = pid[position].Position(pos_error, 1000.f);
			speed_ref = std::max(std::min(speed_ref, (float)maxspeed), -(float)maxspeed);
			current += pid[speed].Delta(speed_ref - curspeed);
		}
	}
	else if (mode == SPD)
	{
		current += pid[speed].Delta(setspeed - curspeed);
	}
	recorded_the_Laps();
	GetDistanceFromMechanicalAngle();
	angle[pre] = angle[now];
	current = setrange(current, maxcurrent);
	odata[trainsmit_or_receive_ID * 2] = (current & 0xff00) >> 8;//高八位
	odata[trainsmit_or_receive_ID * 2 + 1] = current & 0x00ff;
	taskEXIT_CRITICAL();
}
void Motor::recorded_the_Laps() {
	int16_t delta = angle[now] - angle[pre];
	// 处理回绕：顺时针
	if (delta > 8192 / 2)
		delta -= 8192;
	// 处理回绕：逆时针
	else if (delta < -8192 / 2)
		delta += 8192;

	sum_angle+= delta;
//	round_count = total_count / encoder_resolution;
}

uint8_t initial_cnt=0;
void Motor::GetDistanceFromMechanicalAngle() {
	if (initial_cnt<5)
	initial_cnt++;
	distance=(6.2831853f/ 8192.0f)*sum_angle * (WHEEL_RADIUS_MM / GEAR_RATIO)-initial_x;  // 单位：mm

	if(initial_cnt<3)
	initial_x = distance;
}

void Motor::getmax(const type_t type)
{
	adjspeed = 3000;
	switch (type)
	{
	case M3508:
	case M3510:
		maxcurrent = 13000;
		maxspeed = 6000;
		break;
		maxcurrent = 13000;
		maxspeed = 9000;
		break;
	case M2310:
		maxcurrent = 13000;
		maxspeed = 9000;
		adjspeed = 1000;
		break;
	case EC60:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6623:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6020:
		maxcurrent = 15000;
		maxspeed = 80;
		adjspeed = 10;
		break;
	case M2006:
		maxcurrent = 10000;
		adjspeed = 1000;
		maxspeed = 3000;
		break;
	default:;
	}
}

int16_t Motor::getdeltaa(int16_t diff)
{
	if (diff <= -4096)
		diff += 8192;
	else if (diff > 4096)
		diff -= 8192;
	return diff;
}

int16_t Motor::getword(const uint8_t high, const uint8_t low)
{
	const int16_t word = high;
	return (word << 8) + low;
}

int32_t Motor::setrange(const int32_t original, const int32_t range)
{
	return std::max(std::min(range, original), -range);
}

