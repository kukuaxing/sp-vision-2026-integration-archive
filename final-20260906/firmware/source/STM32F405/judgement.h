#pragma once
#include "usart.h"
#include "CRC.h"
#include "string.h"

#define BUFSIZE 100
#define DMA_RX_SIZE 100
#define DMA_TX_SIZE 150
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0         //直线
#define UI_Graph_Rectangle 1    //矩形
#define UI_Graph_Circle 2       //整圆
#define UI_Graph_Ellipse 3      //椭圆
#define UI_Graph_Arc 4          //圆弧
#define UI_Graph_Float 5        //浮点型
#define UI_Graph_Int 6          //整形
#define UI_Graph_Char 7         //字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         //红蓝主色
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //紫红色
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //青色
#define UI_Color_Black 7
#define UI_Color_White 8

class Judgement
{
public:
	bool ready = false;
	bool powerheatready = false;
	bool judgementready = false;
	bool graphInit = false;

	bool capState;

	float prebulletspd = 0;
	uint32_t id_blue = 256;
	uint32_t id_red = 256;

	uint8_t baseRFID;//基地增益
	uint8_t highlandRFID;//高地增益
	uint8_t energyRFID;//能量机关增益
	uint8_t feipoRFID;//飞坡增益
	uint8_t outpostRFID;//前哨站增益
	uint8_t resourseRFID;//资源岛增益
	int32_t nBullet = 0;
	int32_t count = 0;

	//uint8_t mode;
	float voltage;

	void Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base);
	void GetData(void);
	void SendData(void);
	void BuffData();
	void Decode(uint8_t* m_frame);

	void DisplayRP(int flag);
	void DisplayCapState(uint8_t capState);
	void DisplpayMode(uint8_t mode);
	void DisplayCapture(bool isCapture);
	void DisplayCapVoltage(float capVoltage);

	void DisplayStaticUI();
	void DisplayDynamicUI();

	struct {
		uint16_t CmdID;

		//详细描述请翻阅《RoboMaster 2021 裁判系统串口协议附录 V1.0（20210203）》
		//cmd:0x0001 发送频率：1Hz
		struct {
			uint8_t game_type : 4;//比赛类型
			uint8_t game_progress : 4;//当前比赛阶段
			uint16_t stage_remain_time;//当前阶段剩余时间
			uint64_t SyncTimeStamp;
		} game_status_t;

		//cmd:0x0002 比赛结束时发送
		struct {
			uint8_t winner;//0：平局，1：红方胜利，2：蓝方胜利
		} game_result_t;

		//cmd:0x0003 发送频率：1Hz
		struct {
			uint16_t red_1_robot_HP;//红1英雄机器人血量
			uint16_t red_2_robot_HP;//红2工程机器人血量
			uint16_t red_3_robot_HP;//红3步兵机器人血量
			uint16_t red_4_robot_HP;//红4步兵机器人血量
			uint16_t red_5_robot_HP;//红5步兵机器人血量
			uint16_t red_7_robot_HP;//红7哨兵机器人血量
			uint16_t red_outpost_HP;//红方前哨站血量
			uint16_t red_base_HP;//红方基地血量
			uint16_t blue_1_robot_HP;//蓝1英雄机器人血量
			uint16_t blue_2_robot_HP;//蓝2工程机器人血量
			uint16_t blue_3_robot_HP;//蓝3步兵机器人血量
			uint16_t blue_4_robot_HP;//蓝4步兵机器人血量
			uint16_t blue_5_robot_HP;//蓝5步兵机器人血量
			uint16_t blue_7_robot_HP;//蓝7哨兵机器人血量
			uint16_t blue_outpost_HP;//蓝方前哨站血量
			uint16_t blue_base_HP;//蓝方基地血量
		}  game_robot_HP_t;

		//场地事件数据 cmd:0x0101 发送频率：事件改变后发送
		struct
		{
			uint32_t event_data;
		}event_data_t;

		//发送频率：飞镖发射后发送
		struct {
			uint8_t dart_belong;//发射飞镖的队伍；1：红方飞镖，2：蓝方飞镖
			uint16_t stage_remaining_time;//发射时剩余比赛时间
		} ext_dart_status_t;

		//补给站动作标识 cmd:0x0102。发送频率：动作改变后发送, 发送范围：己方机器人
		struct {
			uint8_t reserved;
			uint8_t supply_robot_id;
			uint8_t supply_projectile_step;
			uint8_t supply_projectile_num;
		} ext_supply_projectile_action_t;

		//裁判警告信息 cmd_id:0x0104 发送频率：警告发生后发送
		struct {
			uint8_t level;
			uint8_t foul_robot_id;
			uint8_t count;
		} referee_warning_t;

		//飞镖发射口倒计时 cmd_id:0x0105 发送频率：1Hz 周期发送，发送范围：己方机器人
		struct {
			uint8_t dart_remaining_time;
			uint16_t dart_info;
		}dart_dart_info_t;

		//比赛机器人状态 cmd:0x0201。发送频率：10Hz
		struct {
			uint8_t robot_id;//机器人id
			uint8_t robot_level;//机器人等级
			uint16_t current_HP;//机器人剩余血量
			uint16_t maximum_HP;//机器人上限血量
			uint16_t shooter_barrel_cooling_value;//机器人枪口热量每秒冷却值
			uint16_t shooter_barrel_heat_limit;//机器人枪口热量上限
			uint16_t chassis_power_limit{};//机器人底盘功率上限
			uint8_t power_management_gimbal_output;//gimbal口是否有输出
			uint8_t power_management_chassis_output;//chassis口是否有输出
			uint8_t power_management_shooter_output;//shooter口是否有输出
		}robot_status_t;

		//实时功率热量数据 cmd:0x0202。发送频率：50Hz
		struct {
			uint16_t chassis_voltage;//底盘输出电压 单位毫伏
			uint16_t chassis_current;//底盘输出电流 单位毫安
			float chassis_power;//底盘输出功率 单位瓦
			uint16_t chassis_power_buffer;//底盘功率缓冲
			uint16_t shooter_17mm_1_barrel_heat;//1号17mm枪口热量
			uint16_t shooter_17mm_2_barrel_heat;//2号17mm枪口热量
			uint16_t shooter_42mm_barrel_heat;//42mm枪口热量
		}power_heat_data_t;

		//机器人位置 cmd:0x0203。发送频率：10Hz
		struct {
			float x;//位置x坐标
			float y;//位置y坐标
			float angle;//本机器人测速模块的朝向，单位：度。正北为 0 度
		}robot_pos_t;

		//机器人增益 cmd:0x0204 发送频率：1Hz
		struct {
			uint8_t recovery_buff;//机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
			uint8_t cooling_buff;//机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却
			uint8_t defence_buff;//机器人防御增益（百分比，值为 50 表示 50 % 防御增益）
			uint8_t vulnerability_buff;	//机器人负防御增益（百分比，值为 30 表示 - 30 % 防御增益）
			uint16_t attack_buff;//机器人攻击增益（百分比，值为 50 表示 50 % 攻击增益）
		}buff_t;

		//空中机器人能量状态 cmd:0x0205 发送频率：10Hz
		struct
		{
			uint8_t airforce_status;
			uint8_t time_remain;
		}air_support_data_t;

		//伤害状态 cmd:0x0206 发送频率：伤害发生后发送
		struct
		{
			uint8_t armor_id;
			uint8_t HP_deduction_reason;
		}hurt_data_t;

		//实时射击信息 cmd:0x0207 发送频率：射击后发送
		struct {
			uint8_t bullet_type;
			uint8_t shooter_number;
			uint8_t bullet_freq;//子弹射频 单位Hz
			float bullet_speed;//子弹射速 单位m/s
		}shoot_data_t;

		//子弹剩余发射数及剩余金币数 cmd:0x0208 发送频率：10Hz 周期发送，所有机器人发送
		struct
		{
			uint16_t projectile_allowance_17mm;
			uint16_t projectile_allowance_42mm;
			uint16_t remaining_gold_coin;
		}projectile_allowance_t;

		//机器人 RFID 状态 cmd0:x0209 发送频率：1Hz，发送范围：单一机器人
		struct
		{
			uint32_t rfid_status;
		}rfid_status_t;

		//飞镖机器人客户端指令数据 cmd:0x020A 发送频率：10Hz，发送范围：单一机器人
		struct
		{
			uint8_t dart_launch_opening_status;//当前飞镖发射站的状态
			uint8_t reserved;
			uint16_t target_change_time;//切换击打目标时的比赛剩余时间
			uint16_t latest_launch_cmd_time;//最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0。
		}dart_client_cmd_t;

		//机器人坐标 cmd:0x020B
		struct
		{
			float hero_x;
			float hero_y;
			float engineer_x;
			float engineer_y;
			float standard_3_x;
			float standard_3_y;
			float standard_4_x;
			float standard_4_y;
			float standard_5_x;
			float standard_5_y;
		}ground_robot_position_t;

		//对面机器人标记状态 cmd:0x020C
		struct
		{
			uint8_t mark_hero_progress;
			uint8_t mark_engineer_progress;
			uint8_t mark_standard_3_progress;
			uint8_t mark_standard_4_progress;
			uint8_t mark_standard_5_progress;
			uint8_t mark_sentry_progress;
		}radar_mark_data_t;

		//远程兑换弹丸及复活数量 cmd:0x020D
		struct
		{
			uint32_t sentry_info;
		} sentry_info_t;

		//双倍易伤状态 cmd:0x020E
		struct
		{
			uint8_t radar_info;
		} radar_info_t;

	}data;

#pragma pack(1)
	//-----------------------------------------------------------
	typedef __packed struct
	{
		uint8_t delete_type;
		uint8_t layer;
	}interaction_layer_delete_t;

	typedef  __packed  struct {
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
	}robot_interaction_data_t;
	typedef  __packed  struct {
		uint8_t data[15];
	} robot_interactive_data_t;
	typedef __packed struct
	{
		uint8_t figure_name[3];
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
		uint8_t figure_name[3];
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
		uint8_t show_Data[30] = {};
	} string_data_struct_t;                  //打印字符串数据

	//-----------------------------------------------------------

	typedef __packed  struct
	{
		uint8_t  sof = 0xA5;//数据帧起始字节，固定值为0x05
		uint16_t data_length; //数据中data的长度
		uint8_t  seq;  //包帧头
		uint8_t  crc8;  //帧头CRC8校验

	} frame_header_t;
	typedef __packed  struct
	{
		frame_header_t   						txFrameHeader;
		uint16_t								CMD;//命令字id
		robot_interaction_data_t				txID;
		uint16_t		 						FrameTail;
	}CommunatianData_graphic_t;


#pragma pack()


private:

	uint16_t robotId = UI_Data_RobotID_BStandard3;
	uint16_t clientId = UI_Data_CilentID_BStandard3;

	uint8_t m_uartrx[DMA_RX_SIZE] = { 0 };
	uint8_t m_uarttx[DMA_TX_SIZE] = { 0 };
	uint8_t m_frame[DMA_RX_SIZE] = { 0 };
	uint8_t m_FIFO[BUFSIZE] = { 0 };
	uint8_t* m_whand = m_FIFO;
	uint8_t* m_rhand = m_FIFO;
	uint32_t m_readnum = 0;
	uint32_t m_leftsize = 0;

	uint8_t UI_seq{};

	UART* m_uart;

	union _4bytefloat
	{
		uint8_t b[4];
		float f;
	};
	float u32_to_float(uint8_t* chReceive)
	{
		union _4bytefloat x;
		memcpy(x.b, chReceive, sizeof(float));
		return x.f;
	}

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;
	bool Transmit(uint32_t read_size, uint8_t* plate);


	void LineDraw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
		uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, uint32_t End_x, uint32_t End_y);
	void Rectangle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
		uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, uint32_t End_x, uint32_t End_y);
	void Circle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
		uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
	void Arc_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
		uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
	void Float_Draw(float_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
		uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, float Graph_Float);
	void Char_Draw(string_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer,
		uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, \
		uint32_t Start_y, char* Char_Data);


	void UI_ReFresh(int cnt, graphic_data_struct_t* imagedata);
	void UI_ReFresh(int cnt, float_data_struct_t* floatdata);
	void Char_ReFresh(string_data_struct_t* string_Data);

	void UIDelete(uint8_t deleteOperator, uint8_t deleteLayer);


};

extern "C" Judgement judgement;

