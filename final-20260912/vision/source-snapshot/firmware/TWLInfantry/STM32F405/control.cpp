#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "RC.h"
#include "Heat_limit.h"
#include "xuc_can.h"

#define CTRL_PERIOD_MS          10      // 控制周期，可按实际循环频率调整
#define JAM_CURRENT_THRESHOLD   9000    // 卡弹电流阈值，需结合实机调参
#define JAM_DETECT_TIME_MS      1500    // 超阈值持续该时间后判定卡弹
#define JAM_BACK_TIME_MS        2100    // 卡弹后反转持续时间

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			//supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	if (can1_motor[7].mode == POS) {
		pantile.mark_yaw = para.initial_yaw;
		pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
	}
	else if (can1_motor[7].mode == POS_IMU && xuc.IMUDataValid()) {
		pantile_motor[PANTILE::TYPE::YAW]->setangle = 0.0f;
	}
}


void CONTROL::Control_Pantile(float ch_yaw, float ch_pitch)
{
	ch_pitch *= (-1.f);
    ch_yaw *= (1.f);// 修改方向

	DMmotor[2].setSpeed = 1.5;
	if (can1_motor[7].mode == POS) {
		ctrl.pantile.mark_yaw -= pantile.sensitivity_yaw * ch_yaw * 30.0f;
		if (ctrl.pantile.mark_yaw > 8192.0)ctrl.pantile.mark_yaw -= 8192.0;
		if (ctrl.pantile.mark_yaw < 0.0)ctrl.pantile.mark_yaw += 8192.0;
		can1_motor[7].setangle = ctrl.pantile.mark_yaw;
		DMmotor[2].setPos += ch_pitch * pantile.sensitivity_pitch;
	}
	else if (can1_motor[7].mode == POS_IMU && xuc.IMUDataValid()) {
		if (rc.pc.press_r == 1 && xuc.ControlRequested()) {
			constexpr float RAD2DEG = 57.29577951308232f;
			constexpr float PI_F = 3.14159265358979323846f;

			// XUC协议使用rad；yaw电机的POS_IMU角度环使用degree。
			can1_motor[7].setangle = xuc.GetTargetYaw() * RAD2DEG;

			// 世界系pitch误差为rad。使用实际反馈位置形成绝对位置命令，
			// 避免把同一个目标在每个控制周期重复累加。
			float pitch_error = xuc.GetTargetPitch() - xuc.GetImuPitch();
			while (pitch_error > PI_F) pitch_error -= 2.0f * PI_F;
			while (pitch_error < -PI_F) pitch_error += 2.0f * PI_F;
			DMmotor[2].setPos = DMmotor[2].pos - pitch_error;
		}
		else {        // ctrl.mode == CONTROL::RC || PC
			can1_motor[7].setangle -= pantile.sensitivity_yaw * ch_yaw;
			if (can1_motor[7].setangle > 180.f) {
				can1_motor[7].setangle -= 360.f;
			}
			else if (can1_motor[7].setangle < -180.f) {
				can1_motor[7].setangle += 360.f;
			}
			DMmotor[2].setPos += ch_pitch * pantile.sensitivity_pitch;
		}
	}
	if (DMmotor[2].setPos >= 0.50f) DMmotor[2].setPos = 0.50f;
	if (DMmotor[2].setPos <= -0.25f) DMmotor[2].setPos = -0.25f;
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type)
{
	float delta = 0;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - xuc.GetImuYaw()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;
		if (abs(delta) >= 10.f)
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);
	}
	else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - xuc.GetImuYaw()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;

		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
		}
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{
	double s_x = speedx, s_y = speedy;
	double theat = ctrl.GetDelta(mechanicalToDegree(can1_motor[7].angle[now])
		- mechanicalToDegree(para.initial_yaw)) / 180.f;
	double st = sin(theat*PI);
	double ct = cos(theat*PI);
	speedx = s_x * ct + s_y * st;
	speedy = -s_x * st + s_y * ct;
}

void CONTROL::CHASSIS::Update()
{
	Keep_Direction();

	// 按你的底盘映射顺序：0左前 1右前 2右后 3左后
	ctrl.chassis_motor[0]->setspeed = +speedx + speedy - speedz; // 左前
	ctrl.chassis_motor[1]->setspeed = -speedx + speedy - speedz; // 右前
	ctrl.chassis_motor[2]->setspeed = -speedx - speedy - speedz; // 右后
	ctrl.chassis_motor[3]->setspeed = +speedx - speedy - speedz; // 左后

   // ================= 限位保护 =================
	if (DMmotor[1].setPos > 0.0f)
	{
		DMmotor[1].setPos = 0.0f;
	}
	if (DMmotor[0].setPos < 0.0f)
	{
		DMmotor[0].setPos = 0.0f;
	}
	if (DMmotor[1].setPos < -0.95f)
	{
		DMmotor[1].setPos = -0.95f;
	}
	if (DMmotor[0].setPos > 0.95f)
	{
		DMmotor[0].setPos = 0.95f;
	}
}

void CONTROL::PANTILE::Update()
{

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

}

void CONTROL::SHOOTER::Update()
{
	static int fire_hold_cnt = 0;
	static uint16_t jam_current_cnt = 0;    // 持续过流计数
	static uint16_t jam_back_cnt = 0;       // 反转计数

	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;
	// 摩擦轮控制：由 openRub 标志驱动，不受模式限制
	if (openRub)
	{
		ctrl.shooter_motor[0]->setspeed = shoot_speed;
		ctrl.shooter_motor[1]->setspeed = -shoot_speed;
	}
	else
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
	}

	if (supply_bullet && openRub)
		{
			if (auto_shoot)
			{
				//ctrl.supply_motor[0]->setspeed = 2160;
				//ctrl.supply_motor[0]->spinning = true;
			}
			else
			{
				//ctrl.supply_motor[0]->setspeed = 2160;
				//ctrl.supply_motor[0]->spinning = true;
			}
		}
		else
		{
			//ctrl.supply_motor[0]->spinning = false;
			//ctrl.supply_motor[1]->spinning = false;
		}

	if (fabs(can2_motor[0].curspeed) > 5000 && fabs(can2_motor[1].curspeed) > 5000) {

		if (((rc.pc.press_r == 1 && xuc.fire_auto == 1) || rc.pc.press_l == 1) || rc.rc.go_up == 1) {
			fire_hold_cnt = 30;   // 收到开火请求后保持一段时间
		}

		// 1) 若已处于卡弹反转状态，优先保持反转
		if (ctrl.shooter.jam_block) {
			can1_motor[6].setspeed = -1500;
			jam_back_cnt++;

			if (jam_back_cnt >= (JAM_BACK_TIME_MS / CTRL_PERIOD_MS)) {
				ctrl.shooter.jam_block = false;
				jam_back_cnt = 0;
				jam_current_cnt = 0;
				can1_motor[6].setspeed = 0;
			}
		}
		// 2) 未卡弹时，在保持窗口内执行正转供弹
		else if (fire_hold_cnt > 0 && heatLimiter.CanFire()) {
			fire_hold_cnt--;

			can1_motor[6].setspeed = static_cast<int32_t>(6000.f * heatLimiter.GetFireSpeedScale());

			// 检查电机电流是否超过卡弹阈值
			if (can1_motor[6].current > JAM_CURRENT_THRESHOLD ||
				can1_motor[6].current < -JAM_CURRENT_THRESHOLD) {
				jam_current_cnt++;
			}
			else {
				jam_current_cnt = 0;
			}

			// 过流持续足够时间后进入卡弹反转模式
			if (jam_current_cnt >= (JAM_DETECT_TIME_MS / CTRL_PERIOD_MS)) {
				ctrl.shooter.jam_block = true;
				jam_back_cnt = 0;
				jam_current_cnt = 0;
				can1_motor[6].setspeed = -1500;
			}
		}
		// 3) 无开火请求时停止拨盘电机
		else {
			if (fire_hold_cnt > 0) fire_hold_cnt--;
			can1_motor[6].setspeed = 0;
			jam_current_cnt = 0;
		}
	}
	else
	{
		can1_motor[6].setspeed = 0;
		fire_hold_cnt = 0;
		jam_current_cnt = 0;
		jam_back_cnt = 0;
		ctrl.shooter.jam_block = false;
	}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

extern uint8_t Power_stsRx[];
