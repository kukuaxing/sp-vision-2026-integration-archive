#include "motor.h"
#include "gpio.h"
#include "HTmotor.h"
#include "xuc_can.h"
#include "rc.h"
#include "control.h"
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
void Motor::Ontimer(uint8_t idata[][8], uint8_t* odata)//idate: receive;odate: trainsmit;RC
{
	uint32_t trainsmit_or_receive_ID = this->ID - ID1;

	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID += 4;
	}*/
	//----------------------------------------------------------------
	this->torque_current = getword(idata[trainsmit_or_receive_ID][4], idata[trainsmit_or_receive_ID][5]);
	this->StatusIdentifier(this->torque_current);
	this->angle[now] = getword(idata[trainsmit_or_receive_ID][0], idata[trainsmit_or_receive_ID][1]);
	this->temperature = idata[trainsmit_or_receive_ID][6];
	//Get currrent speed

	motor_status = 0;
	if (temperature > 70) {
		setspeed = 0;
	}
	if (rc.rc.pause_key == 1)  //急停
	{
		can1_motor[0].setspeed = 0;
		can1_motor[1].setspeed = 0;
		can1_motor[2].setspeed = 0;
		can1_motor[3].setspeed = 0;
		can1_motor[4].setspeed = 0;
		can1_motor[5].setspeed = 0;
		can1_motor[6].setspeed = 0;
		can1_motor[7].setspeed = 0;
		can2_motor[0].setspeed = 0;
		can2_motor[1].setspeed = 0;
		can2_motor[2].setspeed = 0;
		can2_motor[3].setspeed = 0;
		can2_motor[4].setspeed = 0;
		can2_motor[5].setspeed = 0;
		can2_motor[6].setspeed = 0;
		can2_motor[7].setspeed = 0;
		DMmotor[0].setSpeed = 0;
		DMmotor[1].setSpeed = 0;
		DMmotor[2].setSpeed = 0;
	}
	if (type == EC60)
	{
		curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
	}
	else {
		curspeed = getword(idata[trainsmit_or_receive_ID][2], idata[trainsmit_or_receive_ID][3]);
	}
	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID -= 4;
	}*/
	//----------------------------------------------------------------
	//20220121--hz
	if (mode == ACE)
	{

		if (spinning)
		{
			//1秒8发 36/1减速比 一圈八格
			current += pid[speed].Delta(setspeed - curspeed);
			current = setrange(current, maxcurrent);
		}
		else {
			if (need_curcircle > 0)
			{
				current += pid[speed].Delta(setspeed - curspeed);
				current = setrange(current, maxcurrent);
				uint8_t deviation = 200;
				if (angle[now] >= stopAngle - deviation && angle[now] <= stopAngle + deviation)
					need_curcircle--;
				if (need_curcircle <= 0)
				{
					need_curcircle = 0;
					stopAngle = angle[now];
					pd = 0;
					setspeed = 0;
					current = 0;
				}
			}
			else if (need_curcircle <= 0)
			{
				setspeed = 0;
				current += pid[speed].Delta(setspeed - curspeed);
				current = setrange(current, maxcurrent);
			}
		}
		if (setspeed == 0 && curspeed == 0)
		{
			motor_status = 1;
			motor_angle_status = angle[0];
		}
		if (motor_status == 1 && fabs(motor_angle_status - angle[0]) < 50)
		{
			current = 0;
		}
	}
	else if (mode == POS)
	{
		float error = getdeltaa(setangle - angle[now]);
		float deg_error = mechanicalToDegree(error);
		setspeed = pid[position].Position(kalman.Filter(deg_error),1000);
		setspeed = setrange(setspeed, maxspeed);
		current = pid[speed].Position(setspeed - curspeed,1000);
		current = currentKalman.Filter(current);
		current = setrange(current, maxcurrent);
	}
	else if (mode == POS_IMU)
	{
		if (!xuc.IMUDataValid())
		{
			setangle = 0.0f;
		}

		constexpr float RAD2DEG = 57.29577951308232f;
		float angle_error =
			getAngleDifference(setangle, xuc.GetImuYaw() * RAD2DEG);
		setspeed = pid[position].Position(kalman.Filter(angle_error), 1000) + K_ROTATION * ctrl.chassis.speedz;
		setspeed = setrange(setspeed, maxspeed);
		current = pid[speed].Position(setspeed - curspeed, 1000);
		current = currentKalman.Filter(current);
		current = setrange(current, maxcurrent);
	}
	else if (mode == SPD)
	{

		current += pid[speed].Delta(setspeed - curspeed);
		current = setrange(current, maxcurrent);
		if (setspeed == 0 && curspeed == 0)
		{
			motor_status = 1;
			motor_angle_status = angle[0];
		}
		if (motor_status == 1&&fabs(motor_angle_status-angle[0])<50)
		{
			current = 0;
		}
	}
	else if (mode == SPD_SHOOT)
	{
		current += pid[speed].Delta(setspeed - curspeed);
		current = setrange(current, maxcurrent);
	}
	recorded_the_Laps();
	GetDistanceFromMechanicalAngle();
	angle[pre] = angle[now];
	current = setrange(current, maxcurrent);
	odata[trainsmit_or_receive_ID * 2] = (current & 0xff00) >> 8;//高八位
	odata[trainsmit_or_receive_ID * 2 + 1] = current & 0x00ff;
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
		maxcurrent = 16384;
		maxspeed = 3800;
		break;
	case M3510:
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
		maxcurrent = 30000;
		maxspeed = 200;
		adjspeed = 80;
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

float Motor::getAngleDifference(float target, float current) {
	float diff = target - current;
	// 归一化到[-180, 180)
	while (diff > 180.0f) diff -= 360.0f;
	while (diff < -180.0f) diff += 360.0f;
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
