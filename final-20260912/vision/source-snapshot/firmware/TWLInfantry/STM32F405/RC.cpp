#include "label.h"
#include "RC.h"
#include "UI.h"
#include "control.h"
#include "HTmotor.h"

#define RC_STATE(s0, s1) ( ((s0) << 8) | (s1) )

namespace
{
float GetChassisSpeedScale(uint8_t modeFlag)
{
    switch (modeFlag)
    {
    case 1:
        return 1.5f;   // aggressive
    case 2:
        return 0.9f;  // conservative
    default:
        return 0.9f;   // normal
    }
}
}
void RC::Decode()
{
    if (queueHandler == NULL || *queueHandler == NULL) {
    	return;  // 队列异常
    }
    else {
    	pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
    }

    if (sizeof(m_frame) < 18) return;
    if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

    rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
    rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
    rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
    rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
    if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
    if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
    if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
    if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

    pre_rc.s[0] = rc.s[0];
    pre_rc.s[1] = rc.s[1];

    rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
    rc.s[1] = ((m_frame[5] >> 4) & 0x03);

    pc.x = m_frame[6] | (m_frame[7] << 8);
    pc.y = m_frame[8] | (m_frame[9] << 8);
    pc.z = m_frame[10] | (m_frame[11] << 8);
    pc.press_l = m_frame[12];
    pc.press_r = m_frame[13];

    pc.key_h = m_frame[15];//按键高 8 位：R F G Z X C
    pc.key_l = m_frame[14];//按键低 8 位：W S A D SHIFT CTRL Q E

    
}

int start_time = 0;
uint8_t go_up_lock = 0;
extern uint8_t flag_shoot;

void RC::OnRC()
{
    /* 根据两个拨码开关的组合确定模式
       上上 → RESET        急停
       中中 → CHASSIS_GIMBAL  底盘+云台移动
       上中 → CHASSIS_ONLY    底盘移动+旋转
       中上 → FIRE_CONTROL    发射控制
       其他 → RESET           急停
    */
    switch (RC_STATE(rc.s[0], rc.s[1]))
    {
    case RC_STATE(UP, UP):                                    // 上上：急停
        ctrl.mode = CONTROL::RESET;
        break;

    case RC_STATE(MID, MID):                                  // 中中：底盘+云台
        ctrl.mode = CONTROL::CHASSIS_GIMBAL;
        break;

    case RC_STATE(UP, MID):                                   // 上中：底盘移动+旋转
        ctrl.mode = CONTROL::CHASSIS_ONLY;
        break;

    case RC_STATE(MID, UP):                                   // 中上：发射控制
        ctrl.mode = CONTROL::FIRE_CONTROL;
        break;

    default:                                                  // 其他档位：急停
        ctrl.mode = CONTROL::RESET;
        break;
    }

    /* 按模式执行控制 */
    switch (ctrl.mode)
    {
    case CONTROL::RESET:
        // 急停：底盘速度归零，摩擦轮关闭，DM电机停转
        ctrl.chassis.speedx = 0;
        ctrl.chassis.speedy = 0;
        ctrl.chassis.speedz = 0;
        ctrl.shooter.openRub = false;
        for (int i = 0; i < 3; i++) {
            DMmotor[i].setSpeed = 0;
        }
        break;

    case CONTROL::CHASSIS_GIMBAL:
        // 左拨杆(ch2/ch3) → 底盘前后左右移动 (can1_motor[0-3])
        ctrl.chassis.speedx = chassis_dir_forward * rc.ch[3] * 4000.f / 660.f;
        ctrl.chassis.speedy = chassis_dir_side * -1 * rc.ch[2] * 4000.f / 660.f;
        ctrl.chassis.speedz = 0;

        // 右拨杆(ch0/ch1) → 云台上下左右 (can1_motor[7] + DMmotor[2])
        ctrl.Control_Pantile(rc.ch[0] * para.yaw_speed / 660.f,
                             rc.ch[1] * para.pitch_speed / 660.f);

        ctrl.shooter.openRub = false;
        break;

    case CONTROL::CHASSIS_ONLY:
        // 左拨杆(ch2/ch3) → 底盘前后左右移动
        ctrl.chassis.speedx = chassis_dir_forward * rc.ch[3] * 4000.f / 660.f;
        ctrl.chassis.speedy = chassis_dir_side * -1 * rc.ch[2] * 4000.f / 660.f;
        // 右拨杆左右(ch0) → 底盘旋转
        ctrl.chassis.speedz = chassis_dir_spin * rc.ch[0] * 4000.f / 660.f;

        ctrl.shooter.openRub = false;
        break;

    case CONTROL::FIRE_CONTROL:
        // 开启摩擦轮 can2_motor[0] 和 [1]
        ctrl.shooter.openRub = true;

        // 放弃底盘移动控制
        ctrl.chassis.speedx = 0;
        ctrl.chassis.speedy = 0;
        ctrl.chassis.speedz = 0;
        break;

    default:
        ctrl.mode = CONTROL::RESET;
        break;
    }

    if (Shift_mode())
    {

    }



    //if (rc.go_up == 1 && key_r_toggle == 0)
    //{
    //    can1_motor[4].setspeed = -7000;
    //    can1_motor[5].setspeed = 7000;
    //}
    //else {
    //    can1_motor[4].setspeed = 0;
    //    can1_motor[5].setspeed = 0;
    //}

    //
    //if (ctrl.mode == CONTROL::RC)
    //{
    //    ctrl.chassis.speedx = rc.ch[2] * para.max_speed / 330.f;
    //    ctrl.chassis.speedy = rc.ch[3] * para.max_speed / 330.f;
    //    ctrl.chassis.speedz = -1.f * rc.wheel * para.max_speed / 330.f;

    //    // ==========================================
    //    //  按键边缘检测 (仅处理状态切换)
    //    // ==========================================

    //    // 左键状态机翻转
    //    if (rc.custom_key_l == 1 && key_l_last == 0) {
    //        key_l_toggle = !key_l_toggle; // 按下一次，状态翻转 (0变1，1变0)
    //    }
    //    key_l_last = rc.custom_key_l;

    //    // 右键状态机翻转
    //    if (rc.custom_key_r == 1 && key_r_last == 0) {
    //        key_r_toggle = !key_r_toggle;
    //        // 右键状态改变时，只需执行一次摩擦轮开关操作
    //        ctrl.shooter.openRub = (key_r_toggle == 1);
    //    }
    //    key_r_last = rc.custom_key_r;

    //    // ==========================================
    //    //  基于状态的持续控制逻辑
    //    // ==========================================

    //    // -- 左键相关逻辑 --
    //    if (key_l_toggle == 1) {
    //        // 左键开启：控制 DM 电机
    //        DMmotor[0].setSpeed = 4.8;
    //        DMmotor[1].setSpeed = 4.8;

    //        DMmotor[0].setPos += (rc.ch[1] * 6) / 660000.f;
    //        DMmotor[1].setPos -= (rc.ch[1] * 6) / 660000.f;
    //    }
    //    else {
    //        // 左键关闭：控制云台
    //        ctrl.Control_Pantile(rc.ch[0] * para.yaw_speed / 660.f,
    //            rc.ch[1] * para.pitch_speed / 660.f);
    //    }

    //    // -- 右键相关逻辑 --
    //    if (key_r_toggle == 1) {
    //        ctrl.shooter.supply_bullet = (rc.go_up == 1);
    //    }
    //    else {
    //        ctrl.shooter.supply_bullet = false;
    //    }

    //}
    //else if (ctrl.mode == CONTROL::PC && rc.pause_key != 1)
    //{
    //    //ctrl.chassis.speedz = para.rota_speed;
    //    //ctrl.chassis.Keep_Direction();
    //    can2_motor[0].setspeed =0;
    //    can2_motor[1].setspeed =0;

    //    //ctrl.Control_Pantile_IMU();
    //    //can1_motor[6].setspeed = 0;
    //}
    //else if (ctrl.mode == CONTROL::AUTOAIM && rc.pause_key != 1)
    //{
    //    //ctrl.Control_Pantile_IMU();
    //    //can2_motor[0].setspeed = -7000;
    //    //can2_motor[1].setspeed = 7000;

    //}

    //if (rc.go_up == 1)  // 上坡
    //{


    //}
    //if (rc.pause_key == 1)  // 暂停
    //{
    //    can1_motor[0].setspeed = 0;
    //    can1_motor[1].setspeed = 0;
    //    can1_motor[2].setspeed = 0;
    //    can1_motor[3].setspeed = 0;
    //    can1_motor[4].setspeed = 0;
    //    can1_motor[5].setspeed = 0;
    //    can1_motor[6].setspeed = 0;
    //    can1_motor[7].setspeed = 0;
    //    can2_motor[0].setspeed = 0;
    //    can2_motor[1].setspeed = 0;
    //    can2_motor[2].setspeed = 0;
    //    can2_motor[3].setspeed = 0;
    //    can2_motor[4].setspeed = 0;
    //    can2_motor[5].setspeed = 0;
    //    can2_motor[6].setspeed = 0;
    //    DMmotor[0].setSpeed = 0;
    //    DMmotor[1].setSpeed = 0;
    //    DMmotor[2].setSpeed = 0;
    //}
}
extern uint8_t flag_shoot;
void RC::OnPC()
{
    //if (ctrl.mode != CONTROL::RC) {
    //    const float chassis_speed_scale = GetChassisSpeedScale(flag_shoot);
    //    const float chassis_speed_limit = para.max_speed * chassis_speed_scale;
    //    const float spin_speed_limit = para.rota_speed * chassis_speed_scale;
    //    const float accel_step = chassis_speed_limit * 0.1f;   // mode-aware ramp
    //    float target_speedx = 0.0f;
    //    float target_speedy = 0.0f;


    //    // 前后目标速度
    //    if (pc.W)
    //        target_speedx = chassis_speed_limit;
    //    else if (pc.S)
    //        target_speedx = -chassis_speed_limit;

    //    // 左右目标速度
    //    if (pc.A)
    //        target_speedy = -chassis_speed_limit;
    //    else if (pc.D)
    //        target_speedy = chassis_speed_limit;

    //    // ---------------- X 轴 ----------------
    //    if (target_speedx == 0.0f)
    //    {
    //        ctrl.chassis.speedx = 0.0f;
    //    }
    //    else if ((ctrl.chassis.speedx > 0.0f && target_speedx < 0.0f) ||
    //        (ctrl.chassis.speedx < 0.0f && target_speedx > 0.0f))
    //    {
    //        ctrl.chassis.speedx = 0.0f;
    //    }
    //    else
    //    {
    //        if (ctrl.chassis.speedx < target_speedx)
    //        {
    //            ctrl.chassis.speedx += accel_step;
    //            if (ctrl.chassis.speedx > target_speedx)
    //                ctrl.chassis.speedx = target_speedx;
    //        }
    //        else if (ctrl.chassis.speedx > target_speedx)
    //        {
    //            ctrl.chassis.speedx -= accel_step;
    //            if (ctrl.chassis.speedx < target_speedx)
    //                ctrl.chassis.speedx = target_speedx;
    //        }
    //    }

    //    // ---------------- Y 轴 ----------------
    //    if (target_speedy == 0.0f)
    //    {
    //        ctrl.chassis.speedy = 0.0f;
    //    }
    //    else if ((ctrl.chassis.speedy > 0.0f && target_speedy < 0.0f) ||
    //        (ctrl.chassis.speedy < 0.0f && target_speedy > 0.0f))
    //    {
    //        ctrl.chassis.speedy = 0.0f;
    //    }
    //    else
    //    {
    //        if (ctrl.chassis.speedy < target_speedy)
    //        {
    //            ctrl.chassis.speedy += accel_step;
    //            if (ctrl.chassis.speedy > target_speedy)
    //                ctrl.chassis.speedy = target_speedy;
    //        }
    //        else if (ctrl.chassis.speedy > target_speedy)
    //        {
    //            ctrl.chassis.speedy -= accel_step;
    //            if (ctrl.chassis.speedy < target_speedy)
    //                ctrl.chassis.speedy = target_speedy;
    //        }
    //    }

    //    // ---------------- Z/C 控制双腿升降（绑定高度） ----------------
    //    static uint8_t leg_height_inited = 0;
    //    static float leg_height = 0.0f;          // 绑定高度：DM0=+h，DM1=-h
    //    constexpr float leg_step = 0.0025f;      // 每次调用步进

    //    if (leg_height_inited == 0)
    //    {
    //        leg_height = (DMmotor[0].setPos - DMmotor[1].setPos) * 0.5f;
    //        if (leg_height < 0.0f) leg_height = 0.0f;
    //        if (leg_height > 0.95f) leg_height = 0.95f;
    //        leg_height_inited = 1;
    //    }

    //    // Z：逐渐上升，C：逐渐下降
    //    if (pc.Z == 1 && pc.C == 0)
    //    {
    //        leg_height += leg_step;
    //    }
    //    else if (pc.C == 1 && pc.Z == 0)
    //    {
    //        leg_height -= leg_step;
    //    }

    //    // 高度限幅
    //    if (leg_height < 0.0f) leg_height = 0.0f;
    //    if (leg_height > 0.95f) leg_height = 0.95f;

    //    DMmotor[0].setSpeed = 4.8f;
    //    DMmotor[1].setSpeed = 4.8f;
    //    DMmotor[0].setPos = leg_height;
    //    DMmotor[1].setPos = -leg_height;

    //    // 限位保护
    //    if (DMmotor[1].setPos > 0.0f) {
    //        DMmotor[1].setPos = 0.0f;
    //    }
    //    if (DMmotor[0].setPos < 0.0f) {
    //        DMmotor[0].setPos = 0.0f;
    //    }
    //    if (DMmotor[1].setPos < -0.95f) {
    //        DMmotor[1].setPos = -0.95f;
    //    }
    //    if (DMmotor[0].setPos > 0.95f) {
    //        DMmotor[0].setPos = 0.95f;
    //    }

    //    //鼠标控制云台
    //    if (pc.x != 0 || pc.y != 0)
    //    {
    //        const float yaw_cmd = pc.x / ctrl.pantile.sensitivity_yaw / 270.0f;
    //        const float pitch_cmd = pc.y/4.0f;
    //        ctrl.Control_Pantile(yaw_cmd, pitch_cmd);
    //    }

    //    // ---------------- X 键切换 speedz ----------------
    //    if (pc.X == 1 && x_last == 0)
    //    {
    //        x_toggle = !x_toggle;
    //    }
    //    x_last = pc.X;

    //    if (x_toggle)
    //    {
    //        ctrl.chassis.speedz = spin_speed_limit;
    //    }
    //    else
    //    {
    //        ctrl.chassis.speedz = (int32_t)((pc.Q - pc.E) * para.max_speed);
    //    }

    //    // ---------------- R 键切换 can1_motor[4][5] 履带----------------
    //    if (pc.R == 1 && r_last == 0)
    //    {
    //    	r_toggle = !r_toggle;
    //    }
    //    r_last = pc.R;


    //    if (r_toggle)
    //    {
    //        ui.displayTrack = true;
    //        can1_motor[4].setspeed = -4000;
    //        can1_motor[5].setspeed = 4000;
    //    }
    //    else
    //    {
    //        ui.displayTrack = false;
    //        can1_motor[4].setspeed = 0;
    //        can1_motor[5].setspeed = 0;
    //    }


    //    // ---------------- F 键切换 can2_motor[0][1] 摩擦轮----------------
    //    if (pc.F == 1 && f_last == 0)
    //    {
    //        f_toggle = !f_toggle;
    //    }
    //    f_last = pc.F;


    //    if (f_toggle)
    //    {
    //        ui.displayOpenRub = true;
    //        can2_motor[0].setspeed = ctrl.shooter.shoot_speed;
    //        can2_motor[1].setspeed = -ctrl.shooter.shoot_speed;
    //    }
    //    else
    //    {
    //        ui.displayOpenRub = false;
    //        can2_motor[0].setspeed = 0;
    //        can2_motor[1].setspeed = 0;
    //    }

    //    
    //    // ---------------- V键切换 flag_shoot (1 <-> 2) ----------------
    //    if (pc.V == 1 && v_last == 0)
    //    {
    //        v_toggle = !v_toggle;

    //        if (v_toggle)
    //            flag_shoot = 1;
    //        else
    //            flag_shoot = 2;
    //    }
    //    v_last = pc.V;
    //    //ctrl.Control_Pantile_IMU();
    //}
}

void RC::Update()
{
    OnRC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
    huart->Init(Instance, BaudRate).DMARxInit(nullptr);
    m_uart = huart;
    queueHandler = &huart->UartQueueHandler;
}

bool RC::Shift_mode()
{
    if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
    {
        return true;
    }
    return false;
}
void RC::Decode_NEW()
{
    // 0~1: 帧头(0xA9 0x53)
    // 2~18: 数据区
    // 19~20: CRC16
    constexpr size_t FRAME_LEN = 21;

    if (queueHandler == NULL || *queueHandler == NULL) return;

    if (xQueueReceive(*queueHandler, m_frame, 0) != pdTRUE) return;

    // 帧头校验
    if (m_frame[0] != 0xA9 || m_frame[1] != 0x53) return;

    // CRC16 校验
    if (!RC::verify_crc16_check_sum(m_frame, FRAME_LEN)) return;

    // ---------------- 通道数据 ----------------
    rc.ch[0] = (int16_t)(((m_frame[2] | ((uint16_t)m_frame[3] << 8)) & 0x07FF) - 1024);
    rc.ch[1] = (int16_t)((((m_frame[3] >> 3) | ((uint16_t)m_frame[4] << 5)) & 0x07FF) - 1024);
    rc.ch[2] = (int16_t)((((m_frame[4] >> 6) | ((uint16_t)m_frame[5] << 2) | ((uint16_t)m_frame[6] << 10)) & 0x07FF) - 1024);
    rc.ch[3] = (int16_t)((((m_frame[6] >> 1) | ((uint16_t)m_frame[7] << 7)) & 0x07FF) - 1024);

    // 死区处理
    if (rc.ch[0] <= 8 && rc.ch[0] >= -8) rc.ch[0] = 0;
    if (rc.ch[1] <= 8 && rc.ch[1] >= -8) rc.ch[1] = 0;
    if (rc.ch[2] <= 8 && rc.ch[2] >= -8) rc.ch[2] = 0;
    if (rc.ch[3] <= 8 && rc.ch[3] >= -8) rc.ch[3] = 0;

    // ---------------- 开关/功能/拨轮 ----------------
    pre_rc.s[0] = rc.s[0];
    pre_rc.s[1] = rc.s[1];

    rc.s[0] = (uint8_t)((m_frame[7] >> 4) & 0x03);   // C:0 N:1 S:2
    rc.s[1] = 0;                                     // 本协议没有第二档开关，默认置 0

    rc.pause_key = (uint8_t)((m_frame[7] >> 6) & 0x01);
    rc.custom_key_l = (uint8_t)((m_frame[7] >> 7) & 0x01);
    rc.custom_key_r = (uint8_t)((m_frame[8] >> 0) & 0x01);
    rc.go_up = (uint8_t)((m_frame[9] >> 4) & 0x01);

    const uint16_t wheel_raw = (uint16_t)((((uint16_t)m_frame[8] >> 1) | ((uint16_t)m_frame[9] << 7)) & 0x07FF);
    rc.wheel = (int16_t)wheel_raw - 1024;
    if (rc.wheel <= 8 && rc.wheel >= -8) rc.wheel = 0;

    // ---------------- 鼠标数据 ----------------
    pc.x = (int16_t)(m_frame[10] | ((uint16_t)m_frame[11] << 8));
    pc.y = (int16_t)(m_frame[12] | ((uint16_t)m_frame[13] << 8));
    pc.z = (int16_t)(m_frame[14] | ((uint16_t)m_frame[15] << 8));

    const uint8_t mb = m_frame[16];
    const uint8_t m_left = (uint8_t)((mb >> 0) & 0x03);
    const uint8_t m_right = (uint8_t)((mb >> 2) & 0x03);
    const uint8_t m_mid = (uint8_t)((mb >> 4) & 0x03);

    pc.press_l = (m_left == 1) ? 1 : 0;
    pc.press_r = (m_right == 1) ? 1 : 0;
    pc.press_m = (m_mid == 1) ? 1 : 0;

    // ---------------- 键盘数据 ----------------
    const uint16_t key = (uint16_t)(m_frame[17] | ((uint16_t)m_frame[18] << 8));

    // 键盘原始值
    //pc.key_l = (uint8_t)(key & 0xFF);
    //pc.key_h = (uint8_t)((key >> 8) & 0xFF);

    // 按键映射：按下=1，未按=0
    pc.W = (key >> 0) & 0x01;
    pc.S = (key >> 1) & 0x01;
    pc.A = (key >> 2) & 0x01;
    pc.D = (key >> 3) & 0x01;
    pc.SHIFT = (key >> 4) & 0x01;
    pc.CTRL = (key >> 5) & 0x01;
    pc.Q = (key >> 6) & 0x01;
    pc.E = (key >> 7) & 0x01;

    pc.R = (key >> 8) & 0x01;
    pc.F = (key >> 9) & 0x01;
    pc.G = (key >> 10) & 0x01;
    pc.Z = (key >> 11) & 0x01;
    pc.X = (key >> 12) & 0x01;
    pc.C = (key >> 13) & 0x01;
    pc.V = (key >> 14) & 0x01;
    pc.B = (key >> 15) & 0x01;
}
