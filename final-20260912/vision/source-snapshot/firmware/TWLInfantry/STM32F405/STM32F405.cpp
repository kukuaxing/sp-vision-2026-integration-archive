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
#include "judgement.h"
#include "led.h"
#include "HTmotor.h"
#include "Power_limit.h"
#include "supercap.h"
#include "xuc_can.h"
#include "Heat_limit.h"
// ========= 你的全局对象不改 =========
Motor can1_motor[CAN1_MOTOR_NUM] = {
    Motor(M3508,SPD,chassis, ID1, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M3508,SPD,chassis, ID2, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M3508,SPD,chassis, ID3, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M3508,SPD,chassis, ID4, PID(10.f, 0.0f, 1.5f,0.f)),
    Motor(M2006,SPD,chassis, ID6, PID(5.f, 0.0f, 1.f,0.f)),//-
    Motor(M2006,SPD,chassis, ID7, PID(5.f, 0.0f, 1.f,0.f)),//+
    Motor(M2006,SPD,supply, ID8, PID(5.f, 0.0f, 1.5f,0.f)),
    Motor(M6020,POS,pantile, ID5, PID(1800.f, 0.0f, 400.0f,0.f),PID(0.8f, 0.005f, 15.0f,0.f))
};
Motor can2_motor[CAN2_MOTOR_NUM] = {
    Motor(M3508,SPD_SHOOT,shooter, ID1, PID(5.f, 0.0f, 1.5f,0.f)),//-
    Motor(M3508,SPD_SHOOT,shooter, ID2, PID(5.f, 0.0f, 1.5f,0.f)),//+
    Motor(M6020,POS,pantile, ID8, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
    Motor(M6020,POS,pantile, ID4, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
    Motor(M6020,POS,pantile, ID7, PID(40.f, 0.0f, 1.5f,0.f),PID(0.8f, 0.005f, 15.0f,0.f)),
    Motor(M6020,SPD,pantile, ID3, PID(10.f, 0.0f, 1.5f,0.f))
};
DMMOTOR DMmotor[5] = {
    DMMOTOR(0x01, POS_DM),
    DMMOTOR(0x02, POS_DM),
    DMMOTOR(0x03, POS_DM),
    DMMOTOR(0x04, POS_DM),
    DMMOTOR(0x05, POS_DM),
};

CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile;
DELAY delay;
RC rc;
LED led1, led2, led3, led4;
TASK task;
CONTROL ctrl;
Judgement judgement;
PARAMETER para;
SUPERCAP supercap;
XUC xuc;
UI ui;

namespace
{
Motor* g_powerMotors[4] = {
    &can1_motor[0],
    &can1_motor[1],
    &can1_motor[2],
    &can1_motor[3],
};
}
// ========= 保留 CAN 队列接口（当前 IMU/XUC 改为串口，暂不启用） =========
QueueHandle_t g_imu_can_queue = NULL;
QueueHandle_t g_xuc_can_queue = NULL;
static void IMU_CanQueueInit()
{
    // g_imu_can_queue = xQueueCreate(16, sizeof(CanRxMsg_t));
    // g_xuc_can_queue = xQueueCreate(16, sizeof(CanRxMsg_t));
}

int main(void)
{
    SystemClockConfig();
    delay.Init(168);
    HAL_Init();

    // 保留 CAN 队列初始化接口（当前串口方案不启用）
    // IMU_CanQueueInit();

    can1.Init(CAN1);
    can2.Init(CAN2);

    timer.Init(BASE, TIM3, 1000).BaseInit();

    //imu_pantile.Init(&uart5, UART5, 115200, CH010);
    xuc.Init(&uart4, UART4, 460800);

    // 保留 IMU/XUC 的 CAN 初始化接口（当前串口方案不启用）
    // imu_pantile.InitCAN(g_imu_can_queue, 0x99, 100.0f, CH010);
    // xuc.InitCAN(g_xuc_can_queue, 0x98, 100.0f);

    rc.Init(&uart3, USART3, 100000);   //921600
    //power.Init(&uart1, USART1, 9600);
    judgement.Init(&uart1, 115200, USART1);
    supercap.Init(&uart5, 115200, UART5);

    para.Init();

    ctrl.Init(std::vector<Motor*>{
        & can2_motor[0],
        & can2_motor[1]
    });
    ctrl.Init(std::vector<Motor*>{
        & can1_motor[0],
        & can1_motor[1],
        & can1_motor[2],
        & can1_motor[3],
        & can1_motor[4],
        & can1_motor[5],
        & can1_motor[6],
        & can1_motor[7]
    });

    powerLimiter.Init(g_powerMotors, 60.0f);
    heatLimiter.Init();
    task.Init();

    for (;;);
}
    
