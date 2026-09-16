#pragma once
#include "stm32f4xx_hal.h"
#include "usart.h"
//#include <cstring>

#include "FreeRTOS.h"
#include <judgement.h>
#include "pid.h"
#include <string.h>
struct TxPacket
{
	uint8_t header = 0x5A;
	uint8_t detect_color : 1;  // 0-red 1-blue
	bool reset_tracker : 1;
	uint8_t reserved : 6;
	float roll;
	float pitch;
	float yaw;
	float aim_x;
	float aim_y;
	float aim_z;
	uint16_t checksum = 0;
}__attribute__((packed));
struct TxPacket_TJ
{
	uint8_t head[2] = { 'S', 'P' };
	//自瞄相关数据
	uint8_t mode_TJ;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
	float yaw_TJ;
	float yaw_vel_TJ;
	float pitch_TJ;
	float pitch_vel_TJ;
	float roll_TJ;
	float roll_vel_TJ;
	float bullet_speed_TJ;
	uint16_t bullet_count_TJ;  // 子弹累计发送次数
	int8_t game_status_NAV;    // 0:未开始 1:开始 2:结束

	// 底盘
	float chassis_vx;
	float chassis_vy;
	float chassis_wz;

	// 赛事状况
	int32_t game_progress;
	int32_t stage_remain_time;
	int32_t ally_outpost_hp;
	uint32_t event_data; // 位含义示例: bit0/1补给区占领；bit3~4小能量机关状态(0未激活/1已激活/2正在激活)；bit5~6大能量机关状态；bit7~8中央高地占领

	//RobotStatus
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

	//Sentry_info
	uint16_t exchanged_projectile_allowance;  // bit0~10: 除远程兑换外，成功兑换的允许发弹量
	uint8_t remote_exchange_projectile_count; // bit11~14: 成功远程兑换允许发弹量次数
	uint8_t remote_exchange_hp_count;         // bit15~18: 成功远程兑换血量次数
	uint8_t can_confirm_revival;              // bit19: 当前是否可确认免费复活(1=可)
	uint8_t can_buy_instant_revival;          // bit20: 当前是否可兑换立即复活(1=可)
	uint16_t instant_revival_cost;            // bit21~30: 当前兑换立即复活所需金币
	uint8_t is_disengaged;                 // bit0: 当前是否脱战(1=是)
	uint16_t team_17mm_exchange_remain;    // bit1~11: 队伍17mm允许发弹量剩余可兑换数
	uint8_t sentry_posture;                // bit12~13: 姿态(1=进攻 2=防御 3=移动)
	uint8_t can_activate_energy_mechanism; // bit14: 己方能量机关当前是否可进入“正在激活”(1=可)

	//Buff
	uint8_t  recovery_buff;      // 回血增益百分比，10表示每秒恢复最大血量10%
	uint16_t cooling_buff;       // 热量冷却增益直接值，x表示额外+x/s
	uint8_t  defence_buff;       // 防御增益百分比，50表示+50%
	uint8_t  vulnerability_buff; // 负防御增益百分比，30表示-30%
	uint16_t attack_buff;        // 攻击增益百分比，50表示+50%
	uint8_t  remaining_energy;   // bit0~6为剩余能量比例反馈位；能量>=50%时通常返回0x80

	//hurt_data 受伤状况
	uint8_t armor_id;            // rawdata[0]低4位: 受伤模块ID(0~15)
	uint8_t HP_deduction_reason; // rawdata[0]高4位: 扣血原因(0=弹丸攻击 1=装甲/超电离线 5=撞击)

	//允许发弹量
	uint16_t projectile_allowance_17mm;     // 17mm允许发弹量
	uint16_t projectile_allowance_42mm;     // 42mm允许发弹量
	uint16_t remaining_gold_coin;           // 剩余金币
	uint16_t projectile_allowance_fortress; // 堡垒增益点提供的17mm储备允许发弹量

	//实时发弹信息
	int16_t supply_speed ;//播弹速度

	//底盘功率与超电数据
	float chassis_power;
	float remain_energy;
	uint16_t buffer_energy;                // 缓冲能量(J)

	//RFID 
	uint32_t rfid_status;   // 主状态位: bit0基地增益点, bit1/2中央高地, bit5~8飞坡, bit18前哨站, bit19/20补给区, 其余见协议表
	uint8_t  rfid_status_2; // 扩展状态位: bit0~5为隧道区域对方侧RFID点
	uint16_t shooter_17mm_1_barrel_heat;   // 17mm射击热量

	//己方血量信息
	uint16_t ally_1_robot_HP;    // 己方1号(英雄)血量
	uint16_t ally_2_robot_HP;    // 己方2号(工程)血量
	uint16_t ally_3_robot_HP;    // 己方3号(步兵)血量
	uint16_t ally_4_robot_HP;    // 己方4号(步兵)血量
	uint16_t reserved;           // 保留
	uint16_t ally_7_robot_HP;    // 己方7号(哨兵)血量
	uint16_t ally_outpost_HP;    // 己方前哨站血量
	uint16_t ally_base_HP;       // 己方基地血量

	uint16_t crc16_TJ;

}__attribute__((packed));

struct RxPacket
{
	uint8_t header = 0xA5;

	uint16_t checksum = 0;
};
struct RxPacket_TJ
{
    uint8_t head[2] = { 'S', 'P' };

};

struct NavToNucFrame
{
	uint8_t head[4] = { 'N', 'A', 'V', 'I' };
	// navi velocity
	float linear_x=0;
	float linear_y=0;
	float angular_z=0;
	int8_t isReached ;  // 0=未到达 1=已到达 -1=导航失败
	uint16_t crc16_TJ;
} __attribute__((packed));



#pragma pack(push, 1)
struct IMU_Send_Frame {
	// 帧头
	uint8_t frame_header = 0xAA;
	// 命令字段
	uint8_t command;
	// 参数区（4个参数）
	uint8_t param0;
	uint8_t param1;
	uint8_t param2;
	uint8_t param3;
	// CRC校验（不包括帧尾）
	uint16_t crc16;
	// 帧尾
	uint8_t frame_end = 0x0D;
};
#pragma pack(pop)

struct OBS
{
	float target_yaw;
	float cur_yaw;
	float target_pitch;
	float cur_pitch;
	float target_pitch_pre;
	float delta_yaw;
	float delta_pitch;
};

class NUC
{
public:

	TxPacket TxNuc;
	RxPacket RxNuc;
	TxPacket_TJ TxNuc_TJ; 
	RxPacket_TJ RxNuc_TJ;
	NavToNucFrame RxNav;

	float feedforward = 1.2f;
	//FTY 25/2/16
	float yaw;
	float pitch;
	float yaw_diff;
	float pitch_diff;
	float distance;
	bool fireadvice = false;
	float v_y;
	//同济
	uint8_t mode_TJ;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
	float yaw_TJ;
	float yaw_vel_TJ;
	float yaw_acc_TJ;
	float pitch_TJ;
	float pitch_vel_TJ;
	float pitch_acc_TJ;
	int8_t state_switch_TJ;  // 姿态状态: 1=Move 2=Attack 3=Defense
	int8_t fire_control_TJ;  // 火控许可: 0/1
  uint8_t protocol_version;
	uint8_t goal_id;
	uint8_t tactical_state;
	uint8_t posture;
	uint8_t fire_policy;
	uint8_t spin_mode;
	uint8_t supercap_mode;
	uint8_t rule_action_type;
	uint16_t ammo_exchange_target_total;
	uint8_t revive_cmd;
	uint8_t remote_ammo_req_inc;
	uint8_t remote_hp_req_inc;
	uint8_t posture_cmd_referee;
	uint8_t activate_energy_confirm;
	uint8_t claim_periodic_ammo;
	uint16_t crc16_TJ;
	OBS obs;
	//Navigation
	
	float speed_x = 0, speed_y = 0, prespeedx, prespeedy;
	float yaw_bias = -2.7;

	float x, y, z;
	float vx, vy, vz;

	bool track_flag;
	bool test_navigation = false;


	float rad_pitch{}, rad_yaw{};
	float aim_x;
	float aim_y;
	float aim_z;


	struct {
		uint8_t game_type;
		uint8_t game_progress = 4;
		uint16_t stage_remain_time = 420;
	} test_game_status;

	struct {
		uint16_t red_1_robot_HP;
		uint16_t red_2_robot_HP;
		uint16_t red_3_robot_HP;
		uint16_t red_4_robot_HP;
		uint16_t red_5_robot_HP;
		uint16_t red_7_robot_HP;
		uint16_t red_outpost_HP = 1400;
		uint16_t red_base_HP;
		uint16_t blue_1_robot_HP;
		uint16_t blue_2_robot_HP;
		uint16_t blue_3_robot_HP;
		uint16_t blue_4_robot_HP;
		uint16_t blue_5_robot_HP;
		uint16_t blue_7_robot_HP;
		uint16_t blue_outpost_HP = 1400;
		uint16_t blue_base_HP;
	}  test_game_robot_HP;

	enum ARMOR_COLOR { RED = 0, BLUE };
	ARMOR_COLOR own_color;

	uint32_t count = 0, navi_count = 0, aim_count = 0;
	void Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate);
	void Decode();
	void Encode();
    void ForwardCalibrationPose(const uint8_t* data, uint16_t size);
	PID autoaim_controller[2];
	bool stm32_fireadvice = false;
	float yaw_pre, yaw_spd, k_feedforward = 0.5f;
	void send_command(uint8_t command, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3);
	uint16_t CRC16_INIT = 0xFFFF;
	uint16_t CRC_TAB[256] = {
		0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
		0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
		0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
		0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
		0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
		0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
		0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
		0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
		0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
		0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
		0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
		0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
		0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
		0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
		0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
		0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
		0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
		0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
		0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
		0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
		0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
		0x3de3, 0x2c6a, 0x1ef1, 0x0f78
	};

	uint16_t getCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
	uint32_t verifyCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength);
	void appendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength);

	//uint8_t predata[48];

private:


	QueueHandle_t* queue_handler = NULL;
	BaseType_t pd_Rx, pd_Tx;
	UART* m_uart;

	uint8_t* frame;
	uint8_t m_frame[UART_MAX_LEN]{};
	uint8_t tx_data[sizeof(TxPacket_TJ)];
    uint8_t calibration_pose_tx[82]{};

	float u8_to_float(uint8_t* p) {
		float s{}; uint8_t ch[4];
		ch[0] = p[3]; ch[1] = p[2]; ch[2] = p[1]; ch[3] = p[0];

		memcpy(&s, p, 4); return s;
	}
	float FR4(uint8_t* p)
	{
		float s;
		memcpy(&s, p, 4);
		return s;
	}

	int test_cont = 0;
};

extern NUC xuc;
