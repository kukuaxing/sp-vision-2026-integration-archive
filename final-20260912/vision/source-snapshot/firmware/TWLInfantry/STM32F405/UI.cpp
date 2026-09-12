// UI.cpp
#include "UI.h"
#include "math.h"
#include "supercap.h"

void UI::Init(UART* huart)
{
    m_uart = huart;
}

void UI::DisplayStaticUI()
{
    string_data_struct_t staticStringUI;

    switch (count % 100)
    {
    case 0:// 准星: XLine + X刻度
    {
        graphic_data_struct_t g[7] = {};
        LineDraw(&g[0], (char*)"XL", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 0, 540, 1920, 540);
        LineDraw(&g[1], (char*)"xA", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 839, 580, 839, 500);
        LineDraw(&g[2], (char*)"xB", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 721, 580, 721, 500);
        LineDraw(&g[3], (char*)"xC", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 600, 580, 600, 500);
        LineDraw(&g[4], (char*)"X1", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1080, 580, 1080, 500);
        LineDraw(&g[5], (char*)"X2", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1200, 580, 1200, 500);
        LineDraw(&g[6], (char*)"X3", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1320, 580, 1320, 500);
        UI_ReFresh(7, g);
        break;
    }
    case 10: // 准星: YLine + Y刻度
    {
        graphic_data_struct_t g[7] = {};
        LineDraw(&g[0], (char*)"YL", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 960, 0, 960, 900);
        LineDraw(&g[1], (char*)"Y1", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1000, 660, 920, 660);
        LineDraw(&g[2], (char*)"Y2", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1000, 780, 920, 780);
        LineDraw(&g[3], (char*)"Y3", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1000, 900, 920, 900);
        LineDraw(&g[4], (char*)"yA", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1000, 420, 920, 420);
        LineDraw(&g[5], (char*)"yB", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1000, 300, 920, 300);
        LineDraw(&g[6], (char*)"yC", UI_Graph_ADD, 0, UI_Color_Cyan, 1, 1000, 180, 920, 180);
        UI_ReFresh(7, g);
        break;
    }
    case 20: // 车体框架 + Track_1+2 + LegLine/LegState/Leg_Start/Leg_End
    {
        graphic_data_struct_t g[7] = {};
        Rectangle_Draw(&g[0], (char*)"CF", UI_Graph_ADD, 0, UI_Color_Yellow, 3, 1661, 690, 1826, 780);
        Circle_Draw(&g[1], (char*)"TK1", UI_Graph_ADD, 0, UI_Color_Main, 3, 1846, 760, 15);
        Circle_Draw(&g[2], (char*)"TK2", UI_Graph_ADD, 0, UI_Color_Main, 3, 1846, 705, 15);
        LineDraw(&g[3], (char*)"LL", UI_Graph_ADD, 0, UI_Color_Purplish_red, 4, 1685, 461, 1845, 461);
        LineDraw(&g[4], (char*)"LT", UI_Graph_ADD, 0, UI_Color_Cyan, 5, 1685, 461, 1685, 461);
        Circle_Draw(&g[5], (char*)"LA", UI_Graph_ADD, 0, UI_Color_Green, 10, 1669, 459, 5);
        Circle_Draw(&g[6], (char*)"LB", UI_Graph_ADD, 0, UI_Color_Purplish_red, 10, 1849, 459, 5);
        UI_ReFresh(7, g);
        break;
    }
    case 30: // Chassis_1~4, Leg_1~2, Block
    {
        graphic_data_struct_t g[7] = {};
        Circle_Draw(&g[0], (char*)"C1", UI_Graph_ADD, 0, UI_Color_Main, 3, 1806, 800, 15);
        Circle_Draw(&g[1], (char*)"C2", UI_Graph_ADD, 0, UI_Color_Main, 3, 1806, 670, 15);
        Circle_Draw(&g[2], (char*)"C3", UI_Graph_ADD, 0, UI_Color_Main, 3, 1676, 670, 15);
        Circle_Draw(&g[3], (char*)"C4", UI_Graph_ADD, 0, UI_Color_Main, 3, 1676, 800, 15);
        Circle_Draw(&g[4], (char*)"LG1", UI_Graph_ADD, 0, UI_Color_Main, 3, 1726, 760, 15);
        Circle_Draw(&g[5], (char*)"LG2", UI_Graph_ADD, 0, UI_Color_Main, 3, 1726, 705, 15);
        Circle_Draw(&g[6], (char*)"BK", UI_Graph_ADD, 0, UI_Color_Green, 2, 960, 540, 50);
        UI_ReFresh(7, g);
        break;
    }
    case 40: // Yaw 文字
    {
        char str[4] = "YAW";
        Char_Draw(&staticStringUI, (char*)"YW", UI_Graph_ADD, 0, UI_Color_Main,
            20, 3, 2, 1659, 628, str);
        Char_ReFresh(&staticStringUI);
        break;
    }
    case 50: // Pitch 文字
    {
        char str[6] = "Pitch";
        Char_Draw(&staticStringUI, (char*)"PT", UI_Graph_ADD, 0, UI_Color_Main,
            20, 5, 2, 1747, 628, str);
        Char_ReFresh(&staticStringUI);
        break;
    }
    case 60: // Rub 文字
    {
        char str[4] = "RUB";
        Char_Draw(&staticStringUI, (char*)"RU", UI_Graph_ADD, 5, UI_Color_Green,
            30, 3, 3, 60, 640, str);
        Char_ReFresh(&staticStringUI);
        break;
    }
    case 70: 
    {
        char str[6] = "SHOOT";
        Char_Draw(&staticStringUI, (char*)"ST", UI_Graph_ADD, 0, UI_Color_Main,
            20, 5, 2, 1700, 587, str);
        Char_ReFresh(&staticStringUI);
        break;
    }
    case 80: 
    {
        char str[6] = "TRACK";
        Char_Draw(&staticStringUI, (char*)"TK", UI_Graph_ADD, 0, UI_Color_Green,
            30, 3, 3, 60, 700, str);
        Char_ReFresh(&staticStringUI);
        break;
    }
    
    case 90: // MODE 文字
    {
        char str[5] = "MODE";
        Char_Draw(&staticStringUI, (char*)"CA", UI_Graph_ADD, 7, UI_Color_Green,
            30, 3, 3, 60, 760, str);
        Char_ReFresh(&staticStringUI);
        break;
    }
    default:
        break;
    }
}



void UI::DisplayMode(uint8_t mode)
{
    char modeChar[7] = {};
    string_data_struct_t modeData = {};
    const uint32_t graphOperate = graphInit ? UI_Graph_Change : UI_Graph_ADD;

    if (mode == 1)
    {
        modeChar[0] = 'S';
        modeChar[1] = 'U';
        modeChar[2] = 'P';
        modeChar[3] = 'E';
        modeChar[4] = 'R';
    }
    else
    {
        modeChar[0] = 'N';
        modeChar[1] = 'O';
        modeChar[2] = 'R';
        modeChar[3] = 'M';
        modeChar[4] = 'A';
        modeChar[5] = 'L';
    }

    Char_Draw(&modeData, (char*)"MD1", graphOperate, 6, UI_Color_Green,
        30, 7, 3, 200, 760, modeChar);
    Char_ReFresh(&modeData);
}

void UI::DisplayRUB(uint8_t mode)
{
    char modeChar[7] = {};
    string_data_struct_t modeData = {};
    const uint32_t graphOperate = graphInit ? UI_Graph_Change : UI_Graph_ADD;

    if (mode == 1)
    {
        modeChar[0] = 'O';
        modeChar[1] = 'P';
        modeChar[2] = 'E';
        modeChar[3] = 'N';
    }
    else
    {
        modeChar[0] = 'C';
        modeChar[1] = 'L';
        modeChar[2] = 'O';
        modeChar[3] = 'S';
        modeChar[4] = 'E';
        modeChar[5] = 'D';
    }

    Char_Draw(&modeData, (char*)"RUB", graphOperate, 6, UI_Color_Green,
        30, 7, 3, 200, 640, modeChar);
    Char_ReFresh(&modeData);
}

void UI::DisplayTrack(uint8_t mode)
{
    char modeChar[7] = {};
    string_data_struct_t modeData = {};
    const uint32_t graphOperate = graphInit ? UI_Graph_Change : UI_Graph_ADD;

    if (mode == 1)
    {
        modeChar[0] = 'O';
        modeChar[1] = 'P';
        modeChar[2] = 'E';
        modeChar[3] = 'N';
    }
    else
    {
        modeChar[0] = 'C';
        modeChar[1] = 'L';
        modeChar[2] = 'O';
        modeChar[3] = 'S';
        modeChar[4] = 'E';
        modeChar[5] = 'D';
    }

    Char_Draw(&modeData, (char*)"TRK", graphOperate, 6, UI_Color_Green,
        30, 7, 3, 200, 700, modeChar);
    Char_ReFresh(&modeData);
}

void UI::DisplayCapEnergyBar(float capEnergy)
{
    const float capEnergyMax = 1764.f;
    const float capEnergyLegacyMax = 100.f;
    const uint32_t leftX = 670;
    const uint32_t rightX = 1250;
    const uint32_t topY = 80;
    const uint32_t bottomY = 130;
    const uint32_t centerX = (leftX + rightX) / 2;
    const uint32_t centerY = (topY + bottomY) / 2;
    const uint32_t innerHalfMax = (rightX - leftX) / 2 - 4;
    const uint32_t innerHeight = (bottomY - topY) - 4;

    float energyPercent = 0.f;
    if (capEnergy > 0.f && capEnergy <= 1.2f)
    {
        energyPercent = capEnergy * 100.f;
    }
    else if (capEnergy >= 0.f && capEnergy <= capEnergyLegacyMax)
    {
        energyPercent = capEnergy;
    }
    else
    {
        energyPercent = capEnergy * 100.f / capEnergyMax;
    }

    if (energyPercent < 0.f)
    {
        energyPercent = 0.f;
    }
    else if (energyPercent > 100.f)
    {
        energyPercent = 100.f;
    }

    const uint32_t fillHalf = (uint32_t)(energyPercent * (float)innerHalfMax / 100.f);

    graphic_data_struct_t capBarUI[2] = {};
    static bool capBarInited = false;
    static uint16_t capBarRefreshCount = 0;
    capBarRefreshCount++;
    const bool forceReAdd = ((capBarRefreshCount % 30) == 0);
    const uint32_t op = (!capBarInited || forceReAdd) ? UI_Graph_ADD : UI_Graph_Change;
    //const uint32_t op = capBarInited ? UI_Graph_Change : UI_Graph_ADD;

    Rectangle_Draw(&capBarUI[0], "caf", op, 1, UI_Color_Green, 2, leftX, topY, rightX, bottomY);
    LineDraw(&capBarUI[1], "cab", op, 1, UI_Color_Yellow, innerHeight, centerX - fillHalf, centerY, centerX + fillHalf, centerY);

    UI_ReFresh(2, capBarUI);
    capBarInited = true;
}

/************************************************电机连接状态更新*************************************************/
// motorId: 0=Yaw, 1=Pitch, 2=Shoot_L, 3=Shoot_R,
//           4=Chassis_1, 5=Chassis_2, 6=Chassis_3, 7=Chassis_4,
//           8=Leg_1, 9=Leg_2, 10=Track_1, 11=Track_2
void UI::UpdateMotorConnection(uint8_t motorId, bool online)
{
    const uint16_t bit = 1 << motorId;
    const bool last = (m_motorOnlineMask & bit) != 0;
    if (online == last) return; // 状态未变，跳过

    if (online) m_motorOnlineMask |= bit;
    else        m_motorOnlineMask &= ~bit;

    // 在线计数
    if (online && !last)
        m_onlineCount++;
    else if (!online && last)
        m_onlineCount--;
    UpdateConnectionCountText(m_onlineCount);

    const uint32_t color = online ? UI_Color_Green : UI_Color_Main;

    switch (motorId)
    {
    case 0:
    {
        string_data_struct_t s = {};
        char str[4] = "YAW";
        Char_Draw(&s, (char*)"YW", UI_Graph_Change, 0, color, 20, 3, 2, 1659, 628, str);
        Char_ReFresh(&s);
        break;
    }
    case 1:
    {
        string_data_struct_t s = {};
        char str[6] = "Pitch";
        Char_Draw(&s, (char*)"PT", UI_Graph_Change, 0, color, 20, 5, 2, 1747, 628, str);
        Char_ReFresh(&s);
        break;
    }
    case 2:
    {
        string_data_struct_t s = {};
        char str[2] = "L";
        Char_Draw(&s, (char*)"SL", UI_Graph_Change, 0, color, 20, 1, 2, 1666, 587, str);
        Char_ReFresh(&s);
        break;
    }
    case 3:
    {
        string_data_struct_t s = {};
        char str[2] = "R";
        Char_Draw(&s, (char*)"SR", UI_Graph_Change, 0, color, 20, 1, 2, 1826, 587, str);
        Char_ReFresh(&s);
        break;
    }
    case 4:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"C1", UI_Graph_Change, 0, color, 3, 1806, 800, 15);
        UI_ReFresh(1, &g);
        break;
    }
    case 5:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"C2", UI_Graph_Change, 0, color, 3, 1806, 670, 15);
        UI_ReFresh(1, &g);
        break;
    }
    case 6:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"C3", UI_Graph_Change, 0, color, 3, 1676, 670, 15);
        UI_ReFresh(1, &g);
        break;
    }
    case 7:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"C4", UI_Graph_Change, 0, color, 3, 1676, 800, 15);
        UI_ReFresh(1, &g);
        break;
    }
    case 8:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"LG1", UI_Graph_Change, 0, color, 3, 1726, 760, 15);
        UI_ReFresh(1, &g);
        break;
    }
    case 9:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"LG2", UI_Graph_Change, 0, color, 3, 1726, 705, 15);
        UI_ReFresh(1, &g);
        break;
    }
    case 10:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"TK1", UI_Graph_Change, 0, color, 3, 1846, 760, 15);
        UI_ReFresh(1, &g);
        break;
    }
    case 11:
    {
        graphic_data_struct_t g = {};
        Circle_Draw(&g, (char*)"TK2", UI_Graph_Change, 0, color, 3, 1846, 705, 15);
        UI_ReFresh(1, &g);
        break;
    }
    default:
        break;
    }
}

/************************************************命中状态更新*************************************************/
void UI::UpdateHitIndicator(bool isHit)
{
    const int8_t cur = isHit ? 1 : 0;
    if (cur == m_lastHitState) return;
    m_lastHitState = cur;

    const uint32_t op = isHit ? UI_Graph_ADD : UI_Graph_Del;

    {
        graphic_data_struct_t g[2] = {};
        LineDraw(&g[0], (char*)"H1", op, 0, UI_Color_Main, 5, 951, 550, 935, 566);
        LineDraw(&g[1], (char*)"H2", op, 0, UI_Color_Main, 5, 970, 550, 986, 566);
        UI_ReFresh(2, g);
    }
    {
        graphic_data_struct_t g[2] = {};
        LineDraw(&g[0], (char*)"H3", op, 0, UI_Color_Main, 5, 951, 531, 935, 515);
        LineDraw(&g[1], (char*)"H4", op, 0, UI_Color_Main, 5, 970, 531, 986, 515);
        UI_ReFresh(2, g);
    }
}

/************************************************卡弹状态更新*************************************************/
void UI::UpdateBlockIndicator(bool isBlocked)
{
    const int8_t cur = isBlocked ? 1 : 0;
    if (cur == m_lastBlockState) return;
    m_lastBlockState = cur;

    graphic_data_struct_t g = {};
    const uint32_t color = isBlocked ? UI_Color_Yellow : UI_Color_Green;
    Circle_Draw(&g, (char*)"BK", UI_Graph_Change, 0, color, 2, 960, 540, 50);
    UI_ReFresh(1, &g);
}

/************************************************腿高度更新*************************************************/
void UI::UpdateLegHeight(float percent)
{
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 1.0f) percent = 1.0f;
    if (fabs(percent - m_lastLegPercent) < 0.005f) return; // 变化小于 0.5% 跳过
    m_lastLegPercent = percent;

    const uint32_t endX = 1685 + static_cast<uint32_t>(percent * 160.0f);
    const uint32_t startColor = (percent == 0.0f) ? UI_Color_Green : UI_Color_Purplish_red;
    const uint32_t endColor = (percent == 1.0f) ? UI_Color_Green : UI_Color_Purplish_red;

    {
        graphic_data_struct_t g = {};
        LineDraw(&g, (char*)"LT", UI_Graph_Change, 0, UI_Color_Cyan, 5, 1685, 461, endX, 461);
        UI_ReFresh(1, &g);
    }
    {
        graphic_data_struct_t g[2] = {};
        Circle_Draw(&g[0], (char*)"LA", UI_Graph_Change, 0, startColor, 10, 1669, 459, 5);
        Circle_Draw(&g[1], (char*)"LB", UI_Graph_Change, 0, endColor, 10, 1849, 459, 5);
        UI_ReFresh(2, g);
    }
}

/************************************************连接数量文本更新*************************************************/
void UI::UpdateConnectionCountText(uint8_t onlineCount)
{
    if (onlineCount == m_lastOnlineCount) return;
    m_lastOnlineCount = onlineCount;

    string_data_struct_t s = {};
    char connStr[6] = {};
    connStr[0] = '0' + (onlineCount / 10);
    connStr[1] = '0' + (onlineCount % 10);
    connStr[2] = '/';
    connStr[3] = '1';
    connStr[4] = '2';
    Char_Draw(&s, (char*)"CN", UI_Graph_Change, 4, UI_Color_Green,
        20, 5, 2, 1634, 854, connStr);
    Char_ReFresh(&s);
}

/************************************************绘制直线*************************************************/
void UI::LineDraw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t End_x, uint32_t End_y)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Line;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制矩形*************************************************/
void UI::Rectangle_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t End_x, uint32_t End_y)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制整圆*************************************************/
void UI::Circle_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t Graph_Radius)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************/
void UI::Arc_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_StartAngle,
    uint32_t Graph_EndAngle, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t x_Length, uint32_t y_Length)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}

/************************************************绘制浮点型数据*************************************************/
void UI::Float_Draw(float_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Size,
    uint32_t Graph_Digit, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    float Graph_Float)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;

    int32_t temp1 = static_cast<int32_t>(Graph_Float * 1000.0f);
    int32_t temp2 = temp1 / 1024;
    image->end_x = temp2;
    image->radius = temp1 - temp2 * 1024; // 1 -> 1.024e-3
}

/************************************************绘制字符型数据*************************************************/
void UI::Char_Draw(string_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Size,
    uint32_t Graph_Digit, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    char* Char_Data)
{
    memset(image, 0, sizeof(*image));
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->Graph_Control.figure_name[2 - i] = imagename[i];

    image->Graph_Control.figure_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.width = Graph_Width;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.start_angle = Graph_Size;
    image->Graph_Control.end_angle = Graph_Digit;

    for (i = 0; i < static_cast<int>(Graph_Digit); i++)
    {
        image->show_Data[i] = *Char_Data;
        Char_Data++;
    }
}

/************************************************UI删除函数*************************************************/
void UI::UIDelete(uint8_t deleteOperator, uint8_t deleteLayer)
{
    uint16_t dataLength;
    CommunatianData_graphic_t UIDeleteData;

    UIDeleteData.txFrameHeader.sof = 0xA5;
    UIDeleteData.txFrameHeader.data_length = 8;
    UIDeleteData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &UIDeleteData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    UIDeleteData.CMD = UI_CMD_Robo_Exchange;
    UIDeleteData.txID.data_cmd_id = UI_Data_ID_Del;
    UIDeleteData.txID.receiver_ID = clientId;
    UIDeleteData.txID.sender_ID = robotId;

    memcpy(m_uarttx + 5, (uint8_t*)&UIDeleteData.CMD, 8);

    m_uarttx[13] = deleteOperator;
    m_uarttx[14] = deleteLayer;

    dataLength = sizeof(CommunatianData_graphic_t) + 2;

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送图形*************************************************/
void UI::UI_ReFresh(int cnt, graphic_data_struct_t* imageData)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length = 6 + cnt * 15;
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    switch (cnt)
    {
    case 1:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
        break;
    case 2:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
        break;
    case 5:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
        break;
    case 7:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
        break;
    default:
        break;
    }

    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, imageData,
        cnt * sizeof(graphic_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        cnt * sizeof(graphic_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送浮点*************************************************/
void UI::UI_ReFresh(int cnt, float_data_struct_t* floatdata)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length =
        6 + cnt * sizeof(graphic_data_struct_t);
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    switch (cnt)
    {
    case 1:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
        break;
    case 2:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
        break;
    case 5:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
        break;
    case 7:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
        break;
    default:
        break;
    }

    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, floatdata,
        cnt * sizeof(graphic_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        cnt * sizeof(graphic_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送字符*************************************************/
void UI::Char_ReFresh(string_data_struct_t* string_Data)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length = 51;
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    graphicData.txID.data_cmd_id = UI_Data_ID_DrawChar;
    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, string_Data, sizeof(string_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        sizeof(string_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

//void UI::DisplayRP(int flag)
//{
//    char RP[2] = {};
//    string_data_struct_t RPData;
//
//    switch (flag)
//    {
//    case 0:
//        RP[0] = 'D';
//        RP[1] = '0';
//        break;
//    case 1:
//        RP[0] = 'D';
//        RP[1] = '1';
//        break;
//    case 2:
//        RP[0] = 'D';
//        RP[1] = '2';
//        break;
//    default:
//        break;
//    }
//
//    if (!graphInit)
//    {
//        Char_Draw(&RPData, (char*)"aa", UI_Graph_ADD, 5, UI_Color_Green,
//            30, 2, 3, 280, 820, RP);
//    }
//    else
//    {
//        Char_Draw(&RPData, (char*)"aa", UI_Graph_Change, 5, UI_Color_Green,
//            30, 2, 3, 280, 820, RP);
//    }
//
//    Char_ReFresh(&RPData);
//}

//void UI::DisplayCapState(uint8_t capState)
//{
//    char capStateChar[5] = {};
//    string_data_struct_t capStateData;
//
//    if (capState == WORKING)
//    {
//        capStateChar[0] = 'W';
//        capStateChar[1] = 'O';
//        capStateChar[2] = 'R';
//        capStateChar[3] = 'K';
//    }
//    else if (capState == DISCHARGE)
//    {
//        capStateChar[0] = 'D';
//        capStateChar[1] = 'I';
//        capStateChar[2] = 'S';
//        capStateChar[3] = 'C';
//        capStateChar[4] = 'H';
//    }
//    else if (capState == SHUT)
//    {
//        capStateChar[0] = 'S';
//        capStateChar[1] = 'H';
//        capStateChar[2] = 'U';
//        capStateChar[3] = 'T';
//    }
//
//    if (!graphInit)
//    {
//        Char_Draw(&capStateData, (char*)"CO1", UI_Graph_ADD, 7, UI_Color_Green,
//            30, 5, 3, 280, 760, capStateChar);
//    }
//    else
//    {
//        Char_Draw(&capStateData, (char*)"CO1", UI_Graph_Change, 7, UI_Color_Green,
//            30, 5, 3, 280, 760, capStateChar);
//    }
//
//    Char_ReFresh(&capStateData);
//}

//void UI::DisplayCapture(bool isCapture)
//{
//    char captureChar[5] = {};
//    string_data_struct_t captureData;
//
//    if (isCapture)
//    {
//        captureChar[0] = 'T';
//        captureChar[1] = 'R';
//        captureChar[2] = 'U';
//        captureChar[3] = 'E';
//    }
//    else
//    {
//        captureChar[0] = 'F';
//        captureChar[1] = 'A';
//        captureChar[2] = 'L';
//        captureChar[3] = 'S';
//        captureChar[4] = 'E';
//    }
//
//    if (!graphInit)
//    {
//        Char_Draw(&captureData, (char*)"CO", UI_Graph_ADD, 5, UI_Color_Green,
//            30, 5, 3, 280, 640, captureChar);
//    }
//    else
//    {
//        Char_Draw(&captureData, (char*)"CO", UI_Graph_Change, 5, UI_Color_Green,
//            30, 5, 3, 280, 640, captureChar);
//    }
//
//    Char_ReFresh(&captureData);
//}

//void UI::DisplayCapVoltage(float capVoltage)
//{
//    constexpr uint32_t kBarLeft = 604;
//    constexpr uint32_t kBarRight = 1316;
//    constexpr uint32_t kBarCenterY = 125;
//    constexpr uint32_t kBarWidth = 42;
//    constexpr float kCapEnergyMax = 2000.0f;
//
//    graphic_data_struct_t voltageData{};
//    float energy = capVoltage;
//
//    if (energy < 0.0f)
//        energy = 0.0f;
//    else if (energy > kCapEnergyMax)
//        energy = kCapEnergyMax;
//
//    const uint32_t voltagePos = kBarLeft +
//        static_cast<uint32_t>(energy * (kBarRight - kBarLeft) / kCapEnergyMax);
//
//    if (!graphInit)
//    {
//        LineDraw(&voltageData, (char*)"VD1", UI_Graph_ADD, 3, UI_Color_Yellow,
//            kBarWidth, kBarLeft, kBarCenterY, voltagePos, kBarCenterY);
//    }
//    else
//    {
//        LineDraw(&voltageData, (char*)"VD1", UI_Graph_Change, 3, UI_Color_Yellow,
//            kBarWidth, kBarLeft, kBarCenterY, voltagePos, kBarCenterY);
//    }
//
//    UI_ReFresh(1, &voltageData);
//}