// UI.h
#pragma once
#include "usart.h"
#include "CRC.h"
#include "string.h"

#define DMA_TX_SIZE  150

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

#pragma pack(1)

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
} string_data_struct_t;

typedef __packed struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} frame_header_t;

typedef __packed struct
{
    frame_header_t          txFrameHeader;
    uint16_t                CMD;
    robot_interaction_data_t txID;
    uint16_t                FrameTail;
} CommunatianData_graphic_t;

#pragma pack()

class UI
{
public:
    void Init(UART* huart);

    // 显示方法
    void DisplayStaticUI();
    // void DisplayCapState(uint8_t capState);
    void DisplayMode(uint8_t mode);
    void DisplayRUB(uint8_t mode);
    void DisplayTrack(uint8_t mode);
    void DisplayCapEnergyBar(float capEnergy);
    // void DisplayCapture(bool isCapture);
    // void DisplayCapVoltage(float capVoltage);
    void UpdateMotorConnection(uint8_t motorId, bool online);
    void UpdateHitIndicator(bool isHit);
    void UpdateBlockIndicator(bool isBlocked);
    void UpdateLegHeight(float percent); // 0.0f ~ 1.0f

    bool graphInit = false;
    uint32_t count = 0;
    bool displayOpenRub = false;
    bool displayTrack = false;
    uint16_t robotId = UI_Data_RobotID_BStandard3;
    uint16_t clientId = UI_Data_CilentID_BStandard3;

private:
    // 绘制原语
    static void LineDraw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
        uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    static void Rectangle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
        uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    static void Circle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width,
        uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
    static void Arc_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle,
        uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x,
        uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
    static void Float_Draw(float_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
        uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
        uint32_t Start_y, float Graph_Float);
    static void Char_Draw(string_data_struct_t* image, char imagename[3], uint32_t Graph_Operate,
        uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size,
        uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x,
        uint32_t Start_y, char* Char_Data);

    // UI 传输
    void UI_ReFresh(int cnt, graphic_data_struct_t* imagedata);
    void UI_ReFresh(int cnt, float_data_struct_t* floatdata);
    void Char_ReFresh(string_data_struct_t* string_Data);
    void UIDelete(uint8_t deleteOperator, uint8_t deleteLayer);
    void UpdateConnectionCountText(uint8_t onlineCount);

    UART*   m_uart = nullptr;
    uint8_t m_uarttx[DMA_TX_SIZE] = {};
    uint8_t UI_seq = 0;

    // 状态缓存，避免重复刷新
    uint16_t m_motorOnlineMask    = 0;       // 初始假定全离线，首次遍历时全部刷新
    int8_t   m_lastHitState       = -1;     // -1 未知, 0 未命中, 1 命中
    int8_t   m_lastBlockState     = -1;     // -1 未知, 0 未卡弹, 1 卡弹
    float    m_lastLegPercent     = -1.0f;  // 上次腿高度百分比
    uint8_t  m_onlineCount        = 0;      // 当前在线电机数
    uint8_t  m_lastOnlineCount    = 0xFF;   // 上次发送的在线数
};

extern UI ui;
