// judgement.h
#pragma once
#include "usart.h"
#include "CRC.h"
#include "string.h"
#include "UI.h"

#define BUFSIZE      100
#define DMA_RX_SIZE  100

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

class Judgement
{
public:
    bool powerheatready = false;
    bool judgementready = false;

    bool capState;

    float    prebulletspd = 0;
    uint32_t id_blue = 256;
    uint32_t id_red = 256;

    uint8_t baseRFID;      // 基地增益
    uint8_t highlandRFID;  // 高地增益
    uint8_t energyRFID;    // 能量机关增益（可自行利用 event_data_t 等）
    uint8_t feipoRFID;     // 飞坡增益
    uint8_t outpostRFID;   // 前哨站增益
    uint8_t resourseRFID;  // 资源岛增益

    int32_t nBullet = 0;

    float voltage;

    void Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base);
    void GetData(void);
    void SendData(void);
    void BuffData();
    void Decode(uint8_t* m_frame);

    struct {
        uint16_t CmdID;

        // cmd:0x0001 发送频率：1Hz
        struct {
            uint8_t  game_type : 4;      // 比赛类型
            uint8_t  game_progress : 4;  // 当前比赛阶段
            uint16_t stage_remain_time;  // 当前阶段剩余时间
            uint64_t SyncTimeStamp;      // UNIX 时间戳
        } game_status_t;

        // cmd:0x0002 比赛结束时发送
        struct {
            uint8_t winner; // 0：平局，1：红方胜利，2：蓝方胜利
        } game_result_t;

        // cmd:0x0003 己方机器人血量 发送频率：3Hz（2026 协议）
        struct {
            uint16_t ally_1_robot_HP;
            uint16_t ally_2_robot_HP;
            uint16_t ally_3_robot_HP;
            uint16_t ally_4_robot_HP;
            uint16_t reserved;
            uint16_t ally_7_robot_HP;
            uint16_t ally_outpost_HP;
            uint16_t ally_base_HP;
        } game_robot_HP_t;

        // 场地事件数据 cmd:0x0101 发送频率：1Hz
        struct {
            uint32_t event_data;
        } event_data_t;

        // 发送频率：飞镖发射后发送 0x0103（这里你原来没解析，可按需补）
        struct {
            uint8_t  dart_belong;
            uint16_t stage_remaining_time;
        } ext_dart_status_t;

        // 补给站动作标识 cmd:0x0102
        struct {
            uint8_t reserved;
            uint8_t supply_robot_id;
            uint8_t supply_projectile_step;
            uint8_t supply_projectile_num;
        } ext_supply_projectile_action_t;

        // 裁判警告信息 cmd_id:0x0104
        struct {
            uint8_t level;
            uint8_t foul_robot_id;
            uint8_t count;
        } referee_warning_t;

        // 飞镖发射口倒计时 cmd_id:0x0105
        struct {
            uint8_t  dart_remaining_time;
            uint16_t dart_info;
        } dart_dart_info_t;

        // 比赛机器人状态 cmd:0x0201。发送频率：10Hz（2026 未改变布局）
        struct {
            uint8_t  robot_id;
            uint8_t  robot_level;
            uint16_t current_HP;
            uint16_t maximum_HP;
            uint16_t shooter_barrel_cooling_value;
            uint16_t shooter_barrel_heat_limit;
            uint16_t chassis_power_limit;
            uint8_t  power_management_gimbal_output;
            uint8_t  power_management_chassis_output;
            uint8_t  power_management_shooter_output;
        } robot_status_t;

        // 实时功率 / 缓冲能量 & 热量数据 cmd:0x0202（2026 版本）
        struct {
            uint16_t reserved0;
            uint16_t reserved1;
            float    reserved2;
            uint16_t buffer_energy;
            uint16_t shooter_17mm_1_barrel_heat;
            uint16_t shooter_42mm_barrel_heat;
        } power_heat_data_t;

        // 机器人位置 cmd:0x0203。发送频率：10Hz
        struct {
            float x;
            float y;
            float angle; // 正北为 0 度
        } robot_pos_t;

        // 机器人增益 cmd:0x0204 发送频率：3Hz（2026 版本）
        struct {
            uint8_t  recovery_buff;
            uint16_t cooling_buff;
            uint8_t  defence_buff;
            uint8_t  vulnerability_buff;
            uint16_t attack_buff;
            uint8_t  remaining_energy;
        } buff_t;

        // 空中机器人能量状态 cmd:0x0205
        struct {
            uint8_t airforce_status;
            uint8_t time_remain;
        } air_support_data_t;

        // 伤害状态 cmd:0x0206
        struct {
            uint8_t armor_id;
            uint8_t HP_deduction_reason;
        } hurt_data_t;

        // 实时射击信息 cmd:0x0207
        struct {
            uint8_t bullet_type;
            uint8_t shooter_number;
            uint8_t bullet_freq;
            float   bullet_speed;
        } shoot_data_t;

        // 子弹剩余发射数及剩余金币数 cmd:0x0208（2026 版本）
        struct {
            uint16_t projectile_allowance_17mm;
            uint16_t projectile_allowance_42mm;
            uint16_t remaining_gold_coin;
            uint16_t projectile_allowance_fortress;
        } projectile_allowance_t;

        // 机器人 RFID 状态 cmd:0x0209（2026 版本）
        struct {
            uint32_t rfid_status;
            uint8_t  rfid_status_2;
        } rfid_status_t;

        // 飞镖机器人客户端指令数据 cmd:0x020A
        struct {
            uint8_t  dart_launch_opening_status;
            uint8_t  reserved;
            uint16_t target_change_time;
            uint16_t latest_launch_cmd_time;
        } dart_client_cmd_t;

        // 机器人坐标 cmd:0x020B（布局未变，最后两个为保留位）
        struct {
            float hero_x;
            float hero_y;
            float engineer_x;
            float engineer_y;
            float standard_3_x;
            float standard_3_y;
            float standard_4_x;
            float standard_4_y;
            float standard_5_x;   // 实际为 reserved1
            float standard_5_y;   // 实际为 reserved2
        } ground_robot_position_t;

        // 对面机器人标记状态 cmd:0x020C（你未解析的话可暂不使用）
        struct {
            uint8_t mark_hero_progress;
            uint8_t mark_engineer_progress;
            uint8_t mark_standard_3_progress;
            uint8_t mark_standard_4_progress;
            uint8_t mark_standard_5_progress;
            uint8_t mark_sentry_progress;
        } radar_mark_data_t;

        // 远程兑换弹丸及复活数量 cmd:0x020D
        struct {
            uint32_t sentry_info;
        } sentry_info_t;

        // 双倍易伤状态 cmd:0x020E
        struct {
            uint8_t radar_info;
        } radar_info_t;

    } data;

private:
    uint8_t  m_uartrx[DMA_RX_SIZE] = { 0 };
    uint8_t  m_frame[DMA_RX_SIZE] = { 0 };
    uint8_t  m_FIFO[BUFSIZE] = { 0 };
    uint8_t* m_whand = m_FIFO;
    uint8_t* m_rhand = m_FIFO;
    uint32_t m_readnum = 0;
    uint32_t m_leftsize = 0;

    UART* m_uart = nullptr;

    union _4bytefloat
    {
        uint8_t b[4];
        float   f;
    };

    float u32_to_float(uint8_t* chReceive)
    {
        _4bytefloat x;
        memcpy(x.b, chReceive, sizeof(float));
        return x.f;
    }

    BaseType_t     pd_Rx = false;
    QueueHandle_t* queueHandler = NULL;

    bool Transmit(uint32_t read_size, uint8_t* plate);
};

extern "C" Judgement judgement;
