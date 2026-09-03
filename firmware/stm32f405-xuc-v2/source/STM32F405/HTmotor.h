#pragma once
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <cmath>
#include"label.h"
#include "can.h"

constexpr auto MAXHIGH = 0.35;//���������չ�߶ȣ�����
constexpr auto MINHIGH = 0.133;
constexpr auto MAXROLL = 15;//����ROLL��Χ����
#define INIT_ANGLE_F 3.979350f//3.595378198//180+26 3.665191429
//#define INIT_ANGLE_B -0.802851442f//-0.48869218//-22�� -0.3839724;-0.523598775598//-30��//1.25423324
#define INIT_ANGLE_B -0.884955f
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03
#define CMD_CLEAR_MODE      0x04
#define MOTOR_MODE			0x100
#define P_MIN -4*PI    // Radians
#define P_MAX 4*PI        
#define V_MIN -10    // Rad/s
#define V_MAX 10
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define C_MAX 40.f
#define C_MIN -40.f
#define T_MIN -18.0f    //����
#define T_MAX 18.0f
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))



#define KT 1.4f
enum { DM_ID1 = 0x01, DM_ID2, DM_ID3, DM_ID4, DM_ID5 };
enum  POSITION { L_F, L_B, R_F, R_B, F_B, PITCH };
enum  FUCTION_MODE { MIT, SPEED, P_S };
typedef enum {
	CAN_PACKET_SET_DUTY = 0, //ռ�ձ�ģʽ
	CAN_PACKET_SET_CURRENT, //������ģʽ
	CAN_PACKET_SET_CURRENT_BRAKE, // ����ɲ��ģʽ
	CAN_PACKET_SET_RPM, // ת��ģʽ
	CAN_PACKET_SET_POS, // λ��ģʽ
	CAN_PACKET_SET_ORIGIN_HERE, //����ԭ��ģʽ
	CAN_PACKET_SET_POS_SPD, //λ���ٶȻ�ģʽ
} CAN_PACKET_ID;

class DMMOTOR
{
public:
	uint32_t ID;
	FUCTION_MODE function;
	POSITION position;

	float angle[2]{}, setAngle{}, deltaAngle{};//��Ϊ�ǰ�װ���ʱ����ʼ��̬�ĵ�������˼нǣ�
	float pos{}, setPos{};//ת��λ�ã�Ҳ�Ƿ��������ĽǶ�
	float curSpeed, setSpeed;
	float current, setCurrent;
	float torque, setTorque;
	float Kp = 10.f;
	float Kd = 0.6f;

	float uint_to_float(int x_int, float x_min, float x_max, int bits);//�����ú���
	int float_to_uint(float x, float x_min, float x_max, int bits);

	void  CanComm_ControlCmd(CAN hcan, uint8_t cmd, uint32_t id);//���ģʽ����
	void  ZeroPosition(CAN hcan, uint32_t id);//У׼��λ��

	void  Motor_Start(CAN hcan, uint32_t id);
	void DMmotorinit();//��ʼ����ʹ�����е��

	//������������õ��ģʽ����λУ׼
	void  Motor_Stop(CAN hcan, uint32_t id);//���ʧ��

	DMMOTOR& State_Decode(CAN hcan, uint8_t odata[][8]);//���벢��������
	void DMmotor_Ontimer(CAN hcan, float f_kp, float f_kd, uint8_t* odata);//�������㣬����������
	void DMmotor_transmit(uint32_t id);//ʹ�ܲ����Ϳ�������

	void SetTorque(float settorque);
	float GetPosition();
	float GetSpeed();
	float GetTorque();

	//���ǶԵ���������п��ƣ�Ҫ��ID 

	DMMOTOR(const uint32_t ID, FUCTION_MODE function, POSITION position) :ID(ID), function(function), position(position) {};//����ӿ�

};

extern DMMOTOR DMmotor[1];
