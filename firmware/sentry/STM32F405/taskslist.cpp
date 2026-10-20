#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"
static CAN* const dm_bus[2] = { &can1, &can2 };   // DMmotor[i] 所在的总线
//extern float Kp = 10;
//extern float Kd = 0.6;
extern int start_flag;
void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

	xTaskCreate((TaskFunction_t)ArmTask,
		(const char*)"ArmTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}
int CNT = 0;
void MotorUpdateTask(void* pvParameters)
{
	TickType_t xlastWakeTime = xTaskGetTickCount();

	while (1)
	{


		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);

		for (auto& motor : can2_motor)motor.Ontimer(can2.data, can2.temp_data);

		for (uint8_t i = 0; i < sizeof(DMmotor) / sizeof(DMmotor[0]); i++)
			DMmotor[i].State_Decode(can2.jointidata)
			.DMmotor_Ontimer(DMmotor[i].Kp, DMmotor[i].Kd, dm_bus[i]->jointpdata[i]);



	vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));//开始执行该任务之后1ms再执行该任务
}
}

void CanTransimtTask(void* pvParameters)
{
	TickType_t xlastWakeTime1 = xTaskGetTickCount();//不知道为什么要从while里面挪出来，这个函数是FreeRTOS的一个函数，获取当前任务的时间戳

	/* 🔴 原本用 (timer.counter++) % 3，而 timer.counter 是【全局变量】，
	 *    在两处各 +1 一次：tim.cpp 的 HAL_TIM_PeriodElapsedCallback（TIM3 @1kHz
	 *    中断）＋ 本任务（也是 1 kHz）。余数步进 = (1000·Δt+1) mod 3 —— 只有
	 *    SysTick 与 TIM3 的相位【恰好】使总步进为 3 的倍数时，才等价于「每 3ms
	 *    各发一帧」；相位一漂（中断被抢占、tick 抖动）就变成「某帧连发两次、
	 *    另一帧间隔 6ms」。对底盘 4×M3508 的电流环 = 控制周期抖动。
	 *    ⇒ 改用本任务私有计数器，只在本任务递增，与任何中断无关。*/
	static uint8_t can_div = 0;

	while (true)
	{
		switch (++can_div % 3)
		{
		case 0:
				DMmotor[0].DMmotor_transmit(can1);
				DMmotor[1].DMmotor_transmit(can2);
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data);
			can2.Transmit(0x200, can2.temp_data);
			break;
		default:
			break;
		}

		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//开始执行该任务之后1ms再执行该任务

	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
		rc.Update();
		ctrl.chassis.Update();
		ctrl.pantile.Update();
		ctrl.shooter.Update();
		vTaskDelay(5);
	}
}


void DecodeTask(void* pvParameters)
{
	/* ---- 上行发送分频（2026-10-15 新增）----
	 *
	 * 原来 xuc.Encode() 在 ArmTask(100 ms) ⇒ 三帧（0x01/0x02/0x81）一起只有 10 Hz。
	 * 现挪到本任务：5 ms 一拍，%4 ⇒ 每 20 ms 一次 = **50 Hz**。
	 *
	 * 为什么先上 50 Hz 而不是 V2 §6.2 建议的 100 Hz：
	 *   三帧合计 106 B（0x01 46 + 0x02 34 + 0x81 26）。921600 baud 下 1 B ≈ 10.85 µs
	 *   ⇒ 串口线上时间 ≈ **1.15 ms**。100 Hz 需每 10 ms 发一次（占线 11.5%），
	 *   50 Hz 只占 **5.75%**，留一半余量给重传与抖动。
	 *   ⚠️ 这是【首次实测值】：上车先看 uart4.tx_busy_reject 是否恒 0、
	 *      DecodeTask 实际周期是否还稳 5 ms，再决定往 100 Hz 提。
	 *
	 * ⚠️ 计数器是本任务的函数内 static —— 必须【只在这里】递增。
	 *    原因见 CanTransimtTask() 里的同类说明。
	 */
	static uint8_t tx_div = 0;

	while (true)
	{
		rc.Decode();
		imu_pantile.Decode();
		imu_chassis.Decode();
		xuc.Decode();

		/* ---- 上行发送（从 ArmTask 挪来）----
		 * 同任务的好处：xuc.Decode() 与 xuc.Encode() 在同一时间轴上，
		 * 抓包/看 watch 时"这一周期收了什么、发的是什么"能直接对上。*/
		// Keep startup/STOP telemetry alive; physical AUTO is explicit in state.bit7.
		if (++tx_div >= 4) { tx_div = 0; xuc.Encode(); }

		vTaskDelay(5);
	}
}

void ArmTask(void* pvParameters)
{
	/* ⚠️ xuc.Encode() 已挪到 DecodeTask（50 Hz）。
	 *   本任务只剩「达妙初始化 + 功率计发送」：
	 *   - DMmotorinit 各含 1 ms 阻塞 delay，100 ms 周期下无所谓；
	 *   - power.Send() 是 9600 波特【阻塞】发送，绝不能和上报混在一个任务里。
	 * 所以 100 ms 的 ArmTask + 512 B 栈（LED_STK_SIZE）现在只干轻活，风险反而降了。*/
	while (true)
	{
		// Startup requests only: DMmotorinit requires physical STOP/RESET and
		// stops emitting FC after operation or a safety latch. Feedback faults
		// require STOP/RESET + MCU restart; this loop cannot undo the latch.
		DMmotor[0].DMmotorinit(can1);
		DMmotor[1].DMmotorinit(can2);
		power.Send();
		vTaskDelay(100);
	}
}





