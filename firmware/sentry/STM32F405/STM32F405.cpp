  /*
   *  /\\\       /\\\  /\\\\            /\\\\  /\\\        /\\\              /\\\\\\\\\            /\\\\\\\\\     /\\\\\\\\\\\   
   *  \///\\\   /\\\/  \/\\\\\\        /\\\\\\ \/\\\       \/\\\            /\\\///////\\\       /\\\////////    /\\\/////////\\\ 
   *     \///\\\\\\/    \/\\\//\\\    /\\\//\\\ \/\\\       \/\\\           \/\\\     \/\\\     /\\\/            \//\\\      \///  
   *        \//\\\\      \/\\\\///\\\/\\\/ \/\\\ \/\\\       \/\\\           \/\\\\\\\\\\\/     /\\\               \////\\\         
   *          \/\\\\      \/\\\  \///\\\/   \/\\\ \/\\\       \/\\\           \/\\\//////\\\    \/\\\                  \////\\\      
   *           /\\\\\\     \/\\\    \///     \/\\\ \/\\\       \/\\\           \/\\\    \//\\\   \//\\\                    \////\\\   
   *          /\\\////\\\   \/\\\             \/\\\ \//\\\      /\\\            \/\\\     \//\\\   \///\\\           /\\\      \//\\\  
   *         /\\\/   \///\\\ \/\\\             \/\\\  \///\\\\\\\\\/             \/\\\      \//\\\    \////\\\\\\\\\ \///\\\\\\\\\\\/   
   *         \///       \///  \///             \///     \/////////               \///        \///        \/////////    \///////////     
  */

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "label.h"
#include "judgement.h"
#include "led.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"

Motor can1_motor[CAN1_MOTOR_NUM] = {
	Motor(M3508,SPD,shooter, ID2, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M3508,SPD,shooter, ID3, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M2006,POS,supply, ID7, PID(10.0f, 0.03f, 0.5f,0.f), PID(1.0f, 0.01f, 0.02f,0.f)),
	Motor(M6020,POS,pantile, ID5, PID(200.0f, 0.08f, 0.0f,0.f),PID(5.0f, 0.0f, 0.5f,0.5f)),
};
	/*Motor(M3508,SPD,chassis, ID1, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M2006,SPD,chassis, ID2, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M6020,POS,pantile, ID3, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID4, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID6, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,SPD,chassis, ID8, PID(10.f, 0.0f, 1.5f,0.f))*/
Motor can2_motor[CAN2_MOTOR_NUM] = {
	Motor(M3508,SPD,chassis, ID1, PID(1.5f, 0.1f, 0.0f,0.f)),
	Motor(M3508,SPD,chassis, ID2, PID(1.5f, 0.1f, 0.0f,0.f)),
	Motor(M3508,SPD,chassis, ID3, PID(1.5f, 0.1f, 0.0f,0.f)),
	Motor(M3508,SPD,chassis, ID4, PID(1.5f, 0.1f, 0.0f,0.f)),
	/*Motor(M3508,SPD,chassis, ID1, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M2006,SPD,chassis, ID2, PID(10.f, 0.0f, 1.5f,0.f)),
	Motor(M6020,POS,pantile, ID3, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID4, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,POS,pantile, ID7, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
	Motor(M6020,SPD,chassis, ID8, PID(10.f, 0.0f, 1.5f,0.f))*/
};
DMMOTOR DMmotor[2] = {
	DMMOTOR(0x09, P_S, Pitch),//pitch
	DMMOTOR(0x06, SPEED, Yaw),//yaw
};




CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile,imu_chassis;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;


int main(void)
{

	can1_motor[2].use_sum_angle = true;
	SystemClockConfig();      // ⚠️ 必须最先：APB1 定时器时钟（84 MHz）由这里确定
	delay.Init(168);
	HAL_Init();

	/* ---- TIM2 1 MHz 微秒自由计数器（V2 协议的 sample_time_us 基准）----
	 * ⚠️ 必须在 SystemClockConfig() 之后（依赖 APB1 定时器时钟就绪）。
	 * ⚠️ 走寄存器直配、不开中断（1 MHz 溢出即每秒 100 万次中断会拖死系统），
	 *    所以它不占任何 NVIC 向量，也不影响现有 5 个任务的中断优先级分配。
	 * ⚠️ 必须早于任何 DMARxInit() —— 之后 IDLE 中断里就会开始打时间戳，
	 *    晚于它则 rx_time_us 有一段恒为 0。
	 * ⚠️ UsInit() 当前是普通成员函数（不是 static），用已有的 timer 实例调用。
	 *    返回 false 说明 84 MHz 无法整除到目标频率（分频会有截断误差），
	 *    此时微秒时基不可用。*/
	if (!timer.UsInit(1000000))
	{
		/* 微秒时基不可用：不致命（下行协议仍能工作，只是 sample_time_us
		 * 退化为毫秒级 ⇒ 对应 V2 的 time_source=2）。uart4.rx_time_us 会恒为 0，
		 * 上层可据此识别。*/
	}

	can1.Init(CAN1);
	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();

	imu_pantile.Init(&uart2, USART2, 115200, CH010);   // 头部，发射头
	imu_pantile.id = 1;
	imu_chassis.Init(&uart5, UART5, 115200, CH010);   // 身体，大yaw桶
	imu_chassis.id = 2;
	
	rc.Init(&uart1, USART1, 100000);
	
	//power.Init(&uart4,UART4,9600);
	xuc.Init(&uart4, UART4, 921600); // V2 A6/6A, UART4 PA0 TX / PA1 RX (see usart.cpp)

	para.Init();
	ctrl.init_dm();

	ctrl.Init(std::vector<Motor*>{
		&can2_motor[0],
			& can2_motor[1],
			& can2_motor[2],
			& can2_motor[3]
	});
	ctrl.Init(std::vector<Motor*>{
		&can1_motor[0],
			& can1_motor[1],
			& can1_motor[2],
			& can1_motor[3]
	});
	//POS模式的PID参数：位置死区
	can1_motor[2].pos_deadband = 3.0f;    // 拨盘：减速 36:1，输出 0.08°
	can1_motor[3].pos_deadband = 0.05f;    // M6020：直驱，0.3°
	task.Init();
	
}





