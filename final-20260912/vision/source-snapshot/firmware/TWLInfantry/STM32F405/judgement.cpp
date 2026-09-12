// judgement.cpp
#include "judgement.h"
#include "label.h"
#include <cstdarg>
#include "imu.h"
#include "control.h"
#include "xuc_can.h"
#include "RC.h"
#include "supercap.h"
#include "HTmotor.h"
extern uint8_t flag_shoot;

//void Judgement::BuffData()
//{
//    //if (m_uart->updateFlag)
//    //{
//    //    m_uart->updateFlag = false;
//
//    if (queueHandler == NULL || *queueHandler == NULL)
//        return;  // 或者报错
//
//    // ★ 必须判断有没有真的收到队列数据
//    if (xQueueReceive(*queueHandler, m_uartrx, 0) != pdPASS)
//        return;
//
//    m_readnum = m_uart->dataDmaNum;
//
//    if ((m_whand + m_readnum) < (m_FIFO + BUFSIZE))
//    {
//        memcpy(m_whand, m_uartrx, m_readnum);
//        m_whand = m_whand + m_readnum;
//    }
//    else if ((m_whand + m_readnum) == (m_FIFO + BUFSIZE))
//    {
//        memcpy(m_whand, m_uartrx, m_readnum);
//        m_whand = m_FIFO;
//    }
//    else
//    {
//        const uint8_t left_size = m_FIFO + BUFSIZE - m_whand;
//        memcpy(m_whand, m_uartrx, left_size);
//        m_whand = m_FIFO;
//        memcpy(m_whand, m_uartrx + left_size, m_readnum - left_size);
//        m_whand = m_FIFO + m_readnum - left_size;
//    }
//    m_leftsize = m_leftsize + m_readnum;
//
//    supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
//    supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
//    //}
//}

// 在 judgement.cpp 顶部（和其他调试变量一起放）加上：
uint16_t judge_debug_len = 0;
uint8_t  judge_debug_b0 = 0;
uint8_t  judge_debug_b1 = 0;
uint8_t  judge_debug_b2 = 0;



void Judgement::BuffData()
{
    if (queueHandler == NULL || *queueHandler == NULL)
        return;

    // ★ 从队列里收一包原始数据到 m_uartrx
    if (xQueueReceive(*queueHandler, m_uartrx, 0) != pdPASS)
        return;

    // ★ 先不要再信 m_uart->dataDmaNum 了
    //   先用队列元素大小（这里假设就是 DMA_RX_SIZE）
    m_readnum = DMA_RX_SIZE;

    // 调试看看队列里来的东西是不是对的
    judge_debug_len = m_readnum;
    judge_debug_b0 = m_uartrx[0];
    judge_debug_b1 = m_uartrx[1];
    judge_debug_b2 = m_uartrx[2];


    supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
    supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
}



void Judgement::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
    huart->Init(uart_base, baud).DMARxInit();
    m_uart = huart;
    queueHandler = &huart->UartQueueHandler;
    ui.Init(huart);
}

// 根据协议自己定义一个最大 data 长度，防止异常长度把缓冲区撑爆
#define JUDGEMENT_MAX_DATA_LEN  200u

// 加几个调试变量（放在 Judgement 类里或者 cpp 顶部加 static/全局）:
uint8_t judge_debug_step = 0;
uint16_t judge_debug_dlen = 0;
uint16_t judge_debug_frame_len = 0;


void Judgement::GetData(void)
{
    // ★ 一进函数立刻标记一下
    judge_debug_step = 1;

    uint16_t len = (uint16_t)m_readnum;
    if (len < 9)
    {
        // 帮你再看一眼当前长度
        judge_debug_dlen = len;
        // 这里就先返回
        return;
    }

    judge_debug_dlen = len;  // 保存一下本帧总长度

    uint16_t idx = 0;

    while (idx + 9 <= len)
    {
        // 1. 找 SOF
        if (m_uartrx[idx] != 0xA5)
        {
            idx++;
            continue;
        }

        judge_debug_step = 2;

        // 2. CRC8
        if (idx + 5 > len)
            break;

        if (!VerifyCRC8CheckSum(&m_uartrx[idx], 5))
        {
            judge_debug_step = 3;
            idx++;
            continue;
        }

        judge_debug_step = 4;

        // 3. data_length
        const uint16_t data_length =
            (uint16_t)m_uartrx[idx + 1] |
            (uint16_t)(m_uartrx[idx + 2] << 8);

        judge_debug_dlen = data_length;

        if (data_length == 0 || data_length > JUDGEMENT_MAX_DATA_LEN)
        {
            judge_debug_step = 5;
            idx++;
            continue;
        }

        const uint16_t frame_length = (uint16_t)(data_length + 9);
        judge_debug_frame_len = frame_length;

        if (idx + frame_length > len)
        {
            judge_debug_step = 6;
            break;
        }

        judge_debug_step = 7;

        if (!VerifyCRC16CheckSum(&m_uartrx[idx], frame_length))
        {
            judge_debug_step = 8;
            idx++;
            continue;
        }

        // 一帧完整 OK
        judge_debug_step = 9;
        Decode(&m_uartrx[idx]);

        idx += frame_length;
    }
}



void Judgement::SendData(void)
{
    ui.robotId = data.robot_status_t.robot_id;
    ui.clientId = ui.robotId | 0x100;

    static uint8_t last_armor = 0;
    static uint8_t hit_persist = 0;
    const uint8_t cur_armor = data.hurt_data_t.armor_id;
    const bool showHit = (hit_persist > 0);
    const uint8_t RUBFlag = (ui.displayOpenRub == 1) ? 1 : 0;
	const uint8_t TrackFlag = (ui.displayTrack == 1) ? 1 : 0;
    const uint8_t modeFlag = (flag_shoot == 1) ? 1 : 0;
    if (ui.count < 200)
    {
        ui.DisplayStaticUI();
    }
    else
    {
        switch (ui.count % 10)
        {
        //case 0: // 电机连接状态: Yaw, Pitch, Shoot_L, Shoot_R
        //    ui.UpdateMotorConnection(0, can1_motor[7].getStatus() == FINE);
        //    ui.UpdateMotorConnection(1, DMmotor[2].err == 0);
        //    ui.UpdateMotorConnection(2, can2_motor[0].getStatus() == FINE);
        //    ui.UpdateMotorConnection(3, can2_motor[1].getStatus() == FINE);
        //    break;
        case 1: // 电机连接状态: Chassis_1~4
            ui.DisplayCapEnergyBar(supercap.Rxsuper.cap_energy);
            break;
        //case 2: // 电机连接状态: Leg_1, Leg_2, Track_1, Track_2
        //    ui.UpdateMotorConnection(4, can1_motor[0].getStatus() == FINE);
        //    ui.UpdateMotorConnection(5, can1_motor[1].getStatus() == FINE);
        //    ui.UpdateMotorConnection(6, can1_motor[2].getStatus() == FINE);
        //    ui.UpdateMotorConnection(7, can1_motor[3].getStatus() == FINE);
        //    break;
        case 3: // 命中指示 — 检测护甲 ID 变化
            ui.DisplayMode(modeFlag);
            break;
        /*case 4:
            ui.UpdateMotorConnection(8, DMmotor[0].err == 0);
            ui.UpdateMotorConnection(9, DMmotor[1].err == 0);
            ui.UpdateMotorConnection(10, can1_motor[4].getStatus() == FINE);
            ui.UpdateMotorConnection(11, can1_motor[5].getStatus() == FINE);
            break;*/
        case 5: // 卡弹指示
            ui.DisplayRUB(RUBFlag);   
            break;
        case 6:
            
            if (cur_armor != 0 && cur_armor != last_armor)
            {
                last_armor = cur_armor;
                hit_persist = 10; // 命中后持续显示约 300ms
            }
            if (hit_persist > 0) hit_persist--;
            ui.UpdateHitIndicator(showHit);
            break;
        case 7:
            ui.UpdateLegHeight((DMmotor[0].setPos - DMmotor[1].setPos) * 0.5f / 0.95f);
            break;
        case 8:
            ui.DisplayTrack(TrackFlag);
            break;
        case 9:
            ui.UpdateBlockIndicator(ctrl.shooter.jam_block);
            break;
        default:
            break;
        }
    }

    if (ui.count > 500)
    {
        ui.graphInit = true;
    }

    ui.count++;
}

void Judgement::Decode(uint8_t* m_frame)
{
    const uint16_t cmdID =
        static_cast<uint16_t>(m_frame[5] | (m_frame[6] << 8));
    data.CmdID = cmdID;
    uint8_t* rawdata = &m_frame[7];

    switch (cmdID)
    {
        // 0x0001 比赛状态
    case 0x0001:
        data.game_status_t.game_type =
            static_cast<uint8_t>(rawdata[0] & 0x0F);
        data.game_status_t.game_progress =
            static_cast<uint8_t>(rawdata[0] >> 4);
        data.game_status_t.stage_remain_time =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        {
            uint64_t ts = 0;
            for (int i = 0; i < 8; ++i)
            {
                ts |= (static_cast<uint64_t>(rawdata[3 + i]) << (8 * i));
            }
            data.game_status_t.SyncTimeStamp = ts;
        }
        break;

        // 0x0002 比赛结果
    case 0x0002:
        data.game_result_t.winner = rawdata[0];
        break;

        // 0x0003 己方血量
    case 0x0003:
        data.game_robot_HP_t.ally_1_robot_HP =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.game_robot_HP_t.ally_2_robot_HP =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.game_robot_HP_t.ally_3_robot_HP =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.game_robot_HP_t.ally_4_robot_HP =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        data.game_robot_HP_t.reserved =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.game_robot_HP_t.ally_7_robot_HP =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.game_robot_HP_t.ally_outpost_HP =
            static_cast<uint16_t>(rawdata[12] | (rawdata[13] << 8));
        data.game_robot_HP_t.ally_base_HP =
            static_cast<uint16_t>(rawdata[14] | (rawdata[15] << 8));
        break;

        // 0x0101 场地事件
    case 0x0101:
        data.event_data_t.event_data =
            static_cast<uint32_t>(rawdata[0]) |
            (static_cast<uint32_t>(rawdata[1]) << 8) |
            (static_cast<uint32_t>(rawdata[2]) << 16) |
            (static_cast<uint32_t>(rawdata[3]) << 24);
        break;

        // 0x0102 补给站动作
    case 0x0102:
        data.ext_supply_projectile_action_t.reserved = rawdata[0];
        data.ext_supply_projectile_action_t.supply_robot_id = rawdata[1];
        data.ext_supply_projectile_action_t.supply_projectile_step = rawdata[2];
        data.ext_supply_projectile_action_t.supply_projectile_num = rawdata[3];
        break;

        // 0x0104 裁判警告
    case 0x0104:
        data.referee_warning_t.level = rawdata[0];
        data.referee_warning_t.foul_robot_id = rawdata[1];
        data.referee_warning_t.count = rawdata[2];
        break;

        // 0x0105 飞镖倒计时
    case 0x0105:
        data.dart_dart_info_t.dart_remaining_time = rawdata[0];
        data.dart_dart_info_t.dart_info =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        break;

        // 0x0201 机器人状态
    case 0x0201:
        data.robot_status_t.robot_id = rawdata[0];
        judgementready = true;
        data.robot_status_t.robot_level =
            rawdata[1];
        data.robot_status_t.current_HP =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.robot_status_t.maximum_HP =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.robot_status_t.shooter_barrel_cooling_value =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        data.robot_status_t.shooter_barrel_heat_limit =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.robot_status_t.chassis_power_limit =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.robot_status_t.power_management_gimbal_output =
            static_cast<uint16_t>(rawdata[12] & 0x01);
        data.robot_status_t.power_management_chassis_output =
            static_cast<uint16_t>((rawdata[12] & 0x02) >> 1);
        data.robot_status_t.power_management_shooter_output =
            static_cast<uint16_t>((rawdata[12] & 0x04) >> 2);
        break;

        // 0x0202 缓冲能量 & 热量
    case 0x0202:
        powerheatready = true;

        data.power_heat_data_t.reserved0 =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.power_heat_data_t.reserved1 =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.power_heat_data_t.reserved2 = u32_to_float(&rawdata[4]);
        data.power_heat_data_t.buffer_energy =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.power_heat_data_t.shooter_17mm_1_barrel_heat =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.power_heat_data_t.shooter_42mm_barrel_heat =
            static_cast<uint16_t>(rawdata[12] | (rawdata[13] << 8));

        supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
        supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
        break;

        // 0x0203 位置
    case 0x0203:
        data.robot_pos_t.x = u32_to_float(&rawdata[0]);
        data.robot_pos_t.y = u32_to_float(&rawdata[4]);
        data.robot_pos_t.angle = u32_to_float(&rawdata[8]);
        break;

        // 0x0204 增益
    case 0x0204:
        data.buff_t.recovery_buff = rawdata[0];
        data.buff_t.cooling_buff =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        data.buff_t.defence_buff = rawdata[3];
        data.buff_t.vulnerability_buff = rawdata[4];
        data.buff_t.attack_buff =
            static_cast<uint16_t>(rawdata[5] | (rawdata[6] << 8));
        data.buff_t.remaining_energy = rawdata[7];
        break;

        // 0x0205 空中能量
    case 0x0205:
        data.air_support_data_t.airforce_status = rawdata[0];
        data.air_support_data_t.time_remain = rawdata[1];
        break;

        // 0x0206 伤害
    case 0x0206:
        data.hurt_data_t.armor_id =
            static_cast<uint8_t>(rawdata[0] & 0x0F);
        data.hurt_data_t.HP_deduction_reason =
            static_cast<uint8_t>(rawdata[0] >> 4);
        break;

        // 0x0207 射击
    case 0x0207:
        data.shoot_data_t.bullet_type = rawdata[0];
        data.shoot_data_t.shooter_number = rawdata[1];
        data.shoot_data_t.bullet_freq = rawdata[2];
        data.shoot_data_t.bullet_speed = u32_to_float(&rawdata[3]);

        if (prebulletspd != data.shoot_data_t.bullet_speed)
        {
            nBullet++;
            prebulletspd = data.shoot_data_t.bullet_speed;
        }
        break;

        // 0x0208 发弹量
    case 0x0208:
        data.projectile_allowance_t.projectile_allowance_17mm =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.projectile_allowance_t.projectile_allowance_42mm =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.projectile_allowance_t.remaining_gold_coin =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.projectile_allowance_t.projectile_allowance_fortress =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        break;

        // 0x0209 RFID
    case 0x0209:
    {
        uint32_t rfid =
            static_cast<uint32_t>(rawdata[0]) |
            (static_cast<uint32_t>(rawdata[1]) << 8) |
            (static_cast<uint32_t>(rawdata[2]) << 16) |
            (static_cast<uint32_t>(rawdata[3]) << 24);

        data.rfid_status_t.rfid_status = rfid;
        data.rfid_status_t.rfid_status_2 = rawdata[4];

        baseRFID =
            static_cast<uint8_t>((rfid & (1u << 0)) ? 1 : 0);
        highlandRFID =
            static_cast<uint8_t>(((rfid & (1u << 1)) || (rfid & (1u << 2))) ? 1 : 0);
        feipoRFID =
            static_cast<uint8_t>(((rfid & (1u << 5)) || (rfid & (1u << 6)) ||
                (rfid & (1u << 7)) || (rfid & (1u << 8))) ? 1 : 0);
        outpostRFID =
            static_cast<uint8_t>((rfid & (1u << 18)) ? 1 : 0);
        resourseRFID =
            static_cast<uint8_t>(((rfid & (1u << 19)) || (rfid & (1u << 20))) ? 1 : 0);

        energyRFID = 0; // 能量机关可从 event_data_t 等其它地方推
        break;
    }

    // 0x020A 飞镖客户端指令
    case 0x020A:
        data.dart_client_cmd_t.dart_launch_opening_status = rawdata[0];
        data.dart_client_cmd_t.reserved = rawdata[1];
        data.dart_client_cmd_t.target_change_time =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.dart_client_cmd_t.latest_launch_cmd_time =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        break;

        // 0x020B 地面机器人位置
    case 0x020B:
        data.ground_robot_position_t.hero_x = u32_to_float(&rawdata[0]);
        data.ground_robot_position_t.hero_y = u32_to_float(&rawdata[4]);
        data.ground_robot_position_t.engineer_x = u32_to_float(&rawdata[8]);
        data.ground_robot_position_t.engineer_y = u32_to_float(&rawdata[12]);
        data.ground_robot_position_t.standard_3_x = u32_to_float(&rawdata[16]);
        data.ground_robot_position_t.standard_3_y = u32_to_float(&rawdata[20]);
        data.ground_robot_position_t.standard_4_x = u32_to_float(&rawdata[24]);
        data.ground_robot_position_t.standard_4_y = u32_to_float(&rawdata[28]);
        data.ground_robot_position_t.standard_5_x = u32_to_float(&rawdata[32]);
        data.ground_robot_position_t.standard_5_y = u32_to_float(&rawdata[36]);
        break;

    default:
        break;
    }
}

bool Judgement::Transmit(uint32_t read_size, uint8_t* plate)
{
    if (m_leftsize < read_size) return false;

    if ((m_rhand + read_size) < (m_FIFO + BUFSIZE))
    {
        memcpy(plate, m_rhand, read_size);
        m_rhand = m_rhand + read_size;
    }
    else if ((m_rhand + read_size) == (m_FIFO + BUFSIZE))
    {
        memcpy(plate, m_rhand, read_size);
        m_rhand = m_FIFO;
    }
    else
    {
        const uint8_t left_size = m_FIFO + BUFSIZE - m_rhand;
        memcpy(plate, m_rhand, left_size);
        memcpy(plate + left_size, m_rhand = m_FIFO,
            read_size - left_size);
        m_rhand = m_FIFO + read_size - left_size;
    }

    m_leftsize = m_leftsize - read_size;
    return true;
}
