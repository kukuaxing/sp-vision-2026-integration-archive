// judgement.h
#pragma once
#include "usart.h"
#include "CRC.h"
#include "string.h"

#define BUFSIZE      100
#define DMA_RX_SIZE  100
#define DMA_TX_SIZE  150

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************内容ID数据********************/
#define UI_Data_ID_Del     0x100
#define UI_Data_ID_Draw1   0x101
#define UI_Data_ID_Draw2   0x102
#define UI_Data_ID_Draw5   0x103
#define UI_Data_ID_Draw7   0x104
#define UI_Data_ID_DrawChar 0x110
#define UI_Data_ID_SentryCmd 0x0120
#define UI_Data_ID_RadarCmd  0x0121
#define UI_Data_ServerID     0x8080
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero       1
#define UI_Data_RobotID_REngineer   2
#define UI_Data_RobotID_RStandard1  3
#define UI_Data_RobotID_RStandard2  4
#define UI_Data_RobotID_RStandard3  5
#define UI_Data_RobotID_RAerial     6
#define UI_Data_RobotID_RSentry     7
#define UI_Data_RobotID_RRadar      9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero       101
#define UI_Data_RobotID_BEngineer   102
#define UI_Data_RobotID_BStandard1  103
#define UI_Data_RobotID_BStandard2  104
#define UI_Data_RobotID_BStandard3  105
#define UI_Data_RobotID_BAerial     106
#define UI_Data_RobotID_BSentry     107
#define UI_Data_RobotID_BRadar      109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero      0x0101
#define UI_Data_CilentID_REngineer  0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial    0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero      0x0165
#define UI_Data_CilentID_BEngineer  0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial    0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer     1
#define UI_Data_Del_ALL       2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD     1
#define UI_Graph_Change  2
#define UI_Graph_Del     3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line      0         // 直线
#define UI_Graph_Rectangle 1         // 矩形
#define UI_Graph_Circle    2         // 整圆
#define UI_Graph_Ellipse   3         // 椭圆
#define UI_Graph_Arc       4         // 圆弧
#define UI_Graph_Float     5         // 浮点型
#define UI_Graph_Int       6         // 整形
#define UI_Graph_Char      7         // 字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main         0      // 红蓝主色
#define UI_Color_Yellow       1
#define UI_Color_Green        2
#define UI_Color_Orange       3
#define UI_Color_Purplish_red 4      // 紫红色
#define UI_Color_Pink         5
#define UI_Color_Cyan         6      // 青色
#define UI_Color_Black        7
#define UI_Color_White        8

class Judgement
{
public:
    bool ready = false;
    bool powerheatready = false;
    bool judgementready = false;
    bool graphInit = false;
    uint32_t power_feedback_seq = 0;

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
    int32_t count = 0;

    float voltage;

    void Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base);
    void GetData(void);
    void SendData(void);
    void BuffData();
    void Decode(uint8_t* m_frame, uint16_t data_length);

    void DisplayRP(int flag);
    void DisplayCapState(uint8_t capState);
    void DisplpayMode(uint8_t mode);
    void DisplayCapture(bool isCapture);
    void DisplayCapVoltage(float capVoltage);

    void DisplayStaticUI();
    void DisplayDynamicUI();   // 你可以在 cpp 里自己实现

    // 发送 0x0301/0x0120 哨兵自主决策指令（姿态切换、确认复活、立即复活、能量机关激活、兑换请求）
    // posture: 1=进攻 2=防御 3=移动，其它值视为无姿态切换
    // exchangeAllowanceTarget: 0xFFFF 表示沿用当前值；否则写入 bit2~12，且会自动钳制为不小于当前成功兑换值
    // increaseRemoteProjectileReq/increaseRemoteHpReq: 1=在当前请求计数基础上 +1（上限15）
    void SendSentryCmd(uint8_t posture,
        uint8_t confirmRevival,
        uint8_t buyInstantRevival,
        uint8_t activateEnergy,
        uint16_t exchangeAllowanceTarget,
        uint8_t increaseRemoteProjectileReq,
        uint8_t increaseRemoteHpReq);

    // 仅发送姿态切换（最常用）
    void SendSentryPosture(uint8_t posture);

    struct {
        uint16_t CmdID;  // 当前已解析帧的 cmd_id

        // cmd:0x0001 比赛状态数据，1Hz，data_length=11
        struct {
            uint8_t  game_type : 4;      // bit0~3 比赛类型: 1=超级对抗赛 2=高校单项赛 3=ICRA AI挑战赛 4=RMUL 3V3 5=RMUL步兵对抗
            uint8_t  game_progress : 4;  // bit4~7 比赛阶段: 0=未开始 1=准备 2=15s自检 3=5s倒计时 4=比赛中 5=结算中
            uint16_t stage_remain_time;  // 当前阶段剩余时间(秒)
            uint64_t SyncTimeStamp;      // UNIX 时间戳(机器人成功NTP同步后有效)
        } game_status_t;

        // cmd:0x0002 比赛结果数据，结束触发，data_length=1
        struct {
            uint8_t winner; // 0=平局 1=红方胜利 2=蓝方胜利
        } game_result_t;

        // cmd:0x0003 机器人血量数据，3Hz，data_length=16
        struct {
            uint16_t ally_1_robot_HP;    // 己方1号(英雄)血量
            uint16_t ally_2_robot_HP;    // 己方2号(工程)血量
            uint16_t ally_3_robot_HP;    // 己方3号(步兵)血量
            uint16_t ally_4_robot_HP;    // 己方4号(步兵)血量
            uint16_t reserved;           // 保留
            uint16_t ally_7_robot_HP;    // 己方7号(哨兵)血量
            uint16_t ally_outpost_HP;    // 己方前哨站血量
            uint16_t ally_base_HP;       // 己方基地血量
        } game_robot_HP_t;

        // cmd:0x0101 场地事件数据，1Hz，data_length=4
        struct {
            uint32_t event_data; // 位含义示例: bit0/1补给区占领；bit3~4小能量机关状态(0未激活/1已激活/2正在激活)；bit5~6大能量机关状态；bit7~8中央高地占领
        } event_data_t;

        // cmd:0x0103(旧字段保留，当前代码未解析)
        struct {
            uint8_t  dart_belong;
            uint16_t stage_remaining_time;
        } ext_dart_status_t;

        // cmd:0x0102 补给站动作，data_length=4
        struct {
            uint8_t reserved;               // 保留
            uint8_t supply_robot_id;        // 补给机器人ID
            uint8_t supply_projectile_step; // 补弹动作阶段
            uint8_t supply_projectile_num;  // 补弹数量
        } ext_supply_projectile_action_t;

        // cmd:0x0104 裁判警告，data_length=3
        struct {
            uint8_t level;         // 判罚等级: 1=双方黄牌 2=黄牌 3=红牌 4=判负
            uint8_t foul_robot_id; // 违规机器人ID(例如红1=1 蓝1=101；双方黄牌/判负时为0)
            uint8_t count;         // 该机器人对应等级累计违规次数
        } referee_warning_t;

        // cmd:0x0105 飞镖发射相关数据，1Hz，data_length=3
        struct {
            uint8_t  dart_remaining_time; // 己方飞镖发射剩余时间(秒)
            uint16_t dart_info;           // bit0~2最近一次命中目标(0默认/1前哨站/2基地固定/3基地随机固定/4基地随机移动/5基地末端移动)
                                         // bit3~5对方最近被击中目标累计次数(0~4)
                                         // bit6~8当前选定击打目标(0默认或前哨站/1基地固定/2基地随机固定/3基地随机移动/4基地末端移动)
        } dart_dart_info_t;

        // cmd:0x0201 机器人状态，10Hz，data_length=13
        struct {
            uint8_t  robot_id;                        // 机器人ID(红方1~11，蓝方101~111)
            uint8_t  robot_level;                     // 机器人等级
            uint16_t current_HP;                      // 当前血量
            uint16_t maximum_HP;                      // 血量上限
            uint16_t shooter_barrel_cooling_value;    // 射击机构每秒热量冷却值
            uint16_t shooter_barrel_heat_limit;       // 射击机构热量上限
            uint16_t chassis_power_limit;             // 底盘功率上限
            uint8_t  power_management_gimbal_output;  // 电源管理输出bit0: 0=无输出 1=24V
            uint8_t  power_management_chassis_output; // 电源管理输出bit1: 0=无输出 1=24V
            uint8_t  power_management_shooter_output; // 电源管理输出bit2: 0=无输出 1=24V
        } robot_status_t;

        // cmd:0x0202 实时底盘缓冲能量和热量，10Hz，data_length=14
        struct {
            uint16_t reserved0;                    // 保留
            uint16_t reserved1;                    // 保留
            float    chassis_power;                // 协议该位置为保留float，部分老代码会当底盘功率用
            uint16_t buffer_energy;                // 缓冲能量(J)
            uint16_t shooter_17mm_1_barrel_heat;   // 17mm射击热量
            uint16_t shooter_42mm_barrel_heat;     // 42mm射击热量
        } power_heat_data_t;

        // cmd:0x0203 机器人位置，详表12字节(命令总表有16字节冲突)
        struct {
            float x;      // 机器人位置x(m)
            float y;      // 机器人位置y(m)
            float angle;  // 测速模块朝向角(度)，正北=0°
        } robot_pos_t;

        // cmd:0x0204 机器人增益和底盘能量数据，3Hz，data_length=8
        struct {
            uint8_t  recovery_buff;      // 回血增益百分比，10表示每秒恢复最大血量10%
            uint16_t cooling_buff;       // 热量冷却增益直接值，x表示额外+x/s
            uint8_t  defence_buff;       // 防御增益百分比，50表示+50%
            uint8_t  vulnerability_buff; // 负防御增益百分比，30表示-30%
            uint16_t attack_buff;        // 攻击增益百分比，50表示+50%
            uint8_t  remaining_energy;   // bit0~6为剩余能量比例反馈位；能量>=50%时通常返回0x80
        } buff_t;

        // cmd:0x0205 空中机器人能量状态，data_length=2
        struct {
            uint8_t airforce_status; // 空中机器人状态(按官方定义枚举)
            uint8_t time_remain;     // 剩余时间(秒)
        } air_support_data_t;

        // cmd:0x0206 伤害状态，伤害触发发送，data_length=1
        struct {
            uint8_t armor_id;            // rawdata[0]低4位: 受伤模块ID(0~15)
            uint8_t HP_deduction_reason; // rawdata[0]高4位: 扣血原因(0=弹丸攻击 1=装甲/超电离线 5=撞击)
        } hurt_data_t;

        // cmd:0x0207 实时射击数据，发弹触发，data_length=7
        struct {
            uint8_t bullet_type;    // 弹丸类型位定义: bit1=17mm bit2=42mm
            uint8_t shooter_number; // 发射机构ID: 1=17mm 3=42mm(2保留)
            uint8_t bullet_freq;    // 弹丸射频(Hz)
            float   bullet_speed;   // 弹丸初速度(m/s)
        } shoot_data_t;

        // cmd:0x0208 允许发弹量，10Hz，详表8字节(命令总表有6字节冲突)
        struct {
            uint16_t projectile_allowance_17mm;     // 17mm允许发弹量
            uint16_t projectile_allowance_42mm;     // 42mm允许发弹量
            uint16_t remaining_gold_coin;           // 剩余金币
            uint16_t projectile_allowance_fortress; // 堡垒增益点提供的17mm储备允许发弹量
        } projectile_allowance_t;

        // cmd:0x0209 RFID模块状态，3Hz，data_length=5
        struct {
            uint32_t rfid_status;   // 主状态位: bit0基地增益点, bit1/2中央高地, bit5~8飞坡, bit18前哨站, bit19/20补给区, 其余见协议表
            uint8_t  rfid_status_2; // 扩展状态位: bit0~5为隧道区域对方侧RFID点
        } rfid_status_t;

        // cmd:0x020A 飞镖选手端指令，3Hz，data_length=6
        struct {
            uint8_t  dart_launch_opening_status; // 发射站状态: 0=已开启 1=关闭 2=正在开启/关闭
            uint8_t  reserved;                   // 保留
            uint16_t target_change_time;         // 最近切换目标时比赛剩余时间(秒)
            uint16_t latest_launch_cmd_time;     // 最近确认发射指令时比赛剩余时间(秒)
        } dart_client_cmd_t;

        // cmd:0x020B 地面机器人位置，1Hz，data_length=40
        struct {
            float hero_x;       // 己方英雄x(m)
            float hero_y;       // 己方英雄y(m)
            float engineer_x;   // 己方工程x(m)
            float engineer_y;   // 己方工程y(m)
            float standard_3_x; // 己方3号步兵x(m)
            float standard_3_y; // 己方3号步兵y(m)
            float standard_4_x; // 己方4号步兵x(m)
            float standard_4_y; // 己方4号步兵y(m)
            float standard_5_x; // 协议保留字段reserved1
            float standard_5_y; // 协议保留字段reserved2
        } ground_robot_position_t;

        // cmd:0x020C 雷达标记进度数据，1Hz，data_length=2
        struct {
            uint16_t mark_progress; // bit0~5: 对方1/2/3/4/空中/哨兵易伤状态；bit6~11: 己方1/2/3/4/空中/哨兵特殊标识状态；bit12~15保留
        } radar_mark_data_t;

        // cmd:0x020D 哨兵自主决策信息同步，1Hz，data_length=6
        struct {
            uint32_t sentry_info;   // 原始字段(4B)
            uint16_t sentry_info_2; // 原始字段(2B)

            // sentry_info 解析结果
            uint16_t exchanged_projectile_allowance;  // bit0~10: 除远程兑换外，成功兑换的允许发弹量
            uint8_t remote_exchange_projectile_count; // bit11~14: 成功远程兑换允许发弹量次数
            uint8_t remote_exchange_hp_count;         // bit15~18: 成功远程兑换血量次数
            uint8_t can_confirm_revival;              // bit19: 当前是否可确认免费复活(1=可)
            uint8_t can_buy_instant_revival;          // bit20: 当前是否可兑换立即复活(1=可)
            uint16_t instant_revival_cost;            // bit21~30: 当前兑换立即复活所需金币

            // sentry_info_2 解析结果
            uint8_t is_disengaged;                 // bit0: 当前是否脱战(1=是)
            uint16_t team_17mm_exchange_remain;    // bit1~11: 队伍17mm允许发弹量剩余可兑换数
            uint8_t sentry_posture;                // bit12~13: 姿态(1=进攻 2=防御 3=移动)
            uint8_t can_activate_energy_mechanism; // bit14: 己方能量机关当前是否可进入“正在激活”(1=可)
        } sentry_info_t;

        // cmd:0x020E 雷达自主决策信息同步，1Hz，data_length=1
        struct {
            uint8_t radar_info; // bit0~1双倍易伤剩余机会(0~2); bit2对方是否正在双倍易伤; bit3~4己方加密等级(1~3); bit5是否可修改密钥
        } radar_info_t;

    } data;

#pragma pack(1)
    //-----------------------------------------------------------
    typedef __packed struct
    {
        uint8_t delete_type;
        uint8_t layer;
    } interaction_layer_delete_t;

    typedef __packed struct {
        uint16_t data_cmd_id;
        uint16_t sender_ID;
        uint16_t receiver_ID;
    } robot_interaction_data_t;

    typedef __packed struct {
        uint8_t data[15];
    } robot_interactive_data_t;

    typedef __packed struct
    {
        uint8_t  figure_name[3];
        uint32_t operate_tpye : 3;
        uint32_t figure_tpye : 3;
        uint32_t layer : 4;
        uint32_t color : 4;
        uint32_t start_angle : 9;
        uint32_t end_angle : 9;
        uint32_t width : 10;
        uint32_t start_x : 11;
        uint32_t start_y : 11;
        uint32_t radius : 10;
        uint32_t end_x : 11;
        uint32_t end_y : 11;
    } graphic_data_struct_t;

    typedef struct
    {
        uint8_t  figure_name[3];
        uint32_t operate_tpye : 3;
        uint32_t figure_tpye : 3;
        uint32_t layer : 4;
        uint32_t color : 4;
        uint32_t start_angle : 9;
        uint32_t end_angle : 9;
        uint32_t width : 10;
        uint32_t start_x : 11;
        uint32_t start_y : 11;
        uint32_t radius : 10;
        uint32_t end_x : 11;
        uint32_t end_y : 11;
    } float_data_struct_t;

    typedef struct
    {
        graphic_data_struct_t Graph_Control;
        uint8_t               show_Data[30] = {};
    } string_data_struct_t;                  // 打印字符串数据

    //-----------------------------------------------------------

    typedef __packed struct
    {
        uint8_t  sof;          // 0xA5
        uint16_t data_length;  // data 区长度
        uint8_t  seq;          // 包序号
        uint8_t  crc8;         // 帧头 CRC8
    } frame_header_t;

    typedef __packed struct
    {
        frame_header_t          txFrameHeader;
        uint16_t                CMD;
        robot_interaction_data_t txID;
        uint16_t                FrameTail;
    } CommunatianData_graphic_t;

#pragma pack()

private:
    uint16_t robotId = UI_Data_RobotID_BStandard3;
    uint16_t clientId = UI_Data_CilentID_BStandard3;

    uint8_t  m_uartrx[DMA_RX_SIZE] = { 0 };
    uint8_t  m_uarttx[DMA_TX_SIZE] = { 0 };
    uint8_t  m_frame[DMA_RX_SIZE] = { 0 };
    uint8_t  m_FIFO[BUFSIZE] = { 0 };
    uint8_t* m_whand = m_FIFO;
    uint8_t* m_rhand = m_FIFO;
    uint32_t m_readnum = 0;
    uint32_t m_leftsize = 0;

    uint8_t UI_seq{};

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

    uint8_t sentry_remote_projectile_req_count = 0;
    uint8_t sentry_remote_hp_req_count = 0;

    bool Transmit(uint32_t read_size, uint8_t* plate);

    void LineDraw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
        uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    void Rectangle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
        uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    void Circle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
        uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
    void Arc_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle,
        uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x,
        uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
    void Float_Draw(float_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
        uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
        uint32_t Start_y, float Graph_Float);
    void Char_Draw(string_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
        uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
        uint32_t Start_y, char* Char_Data);

    void UI_ReFresh(int cnt, graphic_data_struct_t* imagedata);
    void UI_ReFresh(int cnt, float_data_struct_t* floatdata);
    void Char_ReFresh(string_data_struct_t* string_Data);

    void UIDelete(uint8_t deleteOperator, uint8_t deleteLayer);
};

extern "C" Judgement judgement;
