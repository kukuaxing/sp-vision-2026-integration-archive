#pragma once
// Offline replacements for devices/time/RTOS only; packet acceptance runs actual xuc.cpp.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include "firmware/sentry/STM32F405/aim_position_reference.h"
using std::isfinite;
using QueueHandle_t=void*;
using BaseType_t=int;
#define pdMS_TO_TICKS(ms) (ms)
constexpr uint16_t UART_MAX_LEN=320;
inline uint32_t test_now_us=1000000;
inline uint32_t xTaskGetTickCount(){return test_now_us/1000;}
inline uint32_t HAL_GetTick(){return xTaskGetTickCount();}
#define taskENTER_CRITICAL() do {} while(0)
#define taskEXIT_CRITICAL() do {} while(0)
struct USART_TypeDef {};
struct TIM {static uint32_t NowUs(){return test_now_us;}};
struct UART {
  QueueHandle_t UartQueueHandler=this;
  std::vector<uint8_t> rx,tx;uint32_t arrival=0;bool tx_idle=true;
  UART& Init(USART_TypeDef*,uint32_t){return *this;}
  UART& DMARxInit(void* = nullptr){return *this;}UART& DMATxInit(){return *this;}
  bool ReceiveSnapshot(uint8_t* out,uint16_t& n,uint32_t& t){
    if(rx.empty())return false;n=uint16_t(rx.size());t=arrival;
    std::copy(rx.begin(),rx.end(),out);rx.clear();return true;
  }
  bool DMATransmit(uint8_t* p,uint32_t n){tx.assign(p,p+n);return true;}
  bool TxIdle() const { return tx_idle; }
};
#ifndef SENTINEL_IMU_HOST_TEST
struct IMU {
  float roll=5,pitch=10,yaw=20,gx=1,gy=2,gz=3;bool valid=true;
  uint32_t arrival=999000;uint16_t epoch=1;
  float GetAngleRoll(){return roll;}float GetAnglePitch(){return pitch;}float GetAngleYaw(){return yaw;}
  float GetGyroRoll(){return gx;}float GetGyroPitch(){return gy;}float GetGyroYaw(){return gz;}
  bool IsValid(){return valid;}uint32_t GetArrivalTimeUs(){return arrival;}uint16_t GetEpoch(){return epoch;}
};
inline IMU imu_pantile,imu_chassis;
#endif
constexpr int now=1;
struct Motor {
  bool has_feedback=true,angle_latched=true;float setangle=0;float angle[3]{};
  bool FeedbackFresh() const { return has_feedback; }
  uint32_t FeedbackGeneration() const { return 0; }
  static int16_t getdeltaa(int16_t diff){if(diff<=-4096)diff+=8192;else if(diff>4096)diff-=8192;return diff;}
};
struct DMMOTOR {bool decoded=false;uint8_t status=0;float pos=0,setPos=0,setSpeed=0;
  bool FeedbackFresh() const { return decoded; }
  bool DriveReady() const { return decoded && status == 1; }
  uint32_t FeedbackGeneration() const { return 0; }};
inline DMMOTOR DMmotor[2];
inline struct {float yaw_center=8192,yaw_span=1707,dm_pitch_min=-.55f,dm_pitch_max=.4f;} para;
#define LIMIT_MIN_MAX(x,min,max) (x)=((x)<=(min)?(min):((x)>=(max)?(max):(x)))
struct CONTROL {
  enum {AUTO=1,STOP=2,MANUAL=3,RESET=4,FIRE=5};int mode=AUTO;
  static float ClampAngle(float,float,float);
  struct PANTILE {
    enum TYPE {YAW=0,PITCH=1};float mark_pitch=0,mark_yaw=0,yaw_speed_out=0;
    bool hold_position=false,pitch_ready=false,yaw_lock_ready=false;
    bool hold_yaw_captured=false,hold_pitch_captured=false,hold_dm_yaw_captured=false;
    AimPositionReference aim_yaw_reference,aim_pitch_reference;
    uint32_t yaw_feedback_generation=0,pitch_feedback_generation=0,dm_yaw_feedback_generation=0;
    void Synchronize_Feedback_Recovery();
    void Reset_Aim_References(){aim_yaw_reference.Reset();aim_pitch_reference.Reset();}
    float pitch0=0,yaw_lock_pos=0,yaw_lock_Kp=3;
    void Hold_Pantile();void Control_Pantile(int32_t,int32_t,int32_t);void Update();
  } pantile;
  struct {float speedx=0,speedy=0,speedz=0;} chassis;
  Motor* pantile_motor[2]{};
  struct {bool openRub=false,auto_shoot=false,auto_fire_request_level=false;int trig_raw=0;
    void CancelPendingFeed() {} // Actual feeder/Motor semantics run actuator_safety_test.
  } shooter;
};
inline CONTROL ctrl;
constexpr float PI=3.14159265358979323846f;
#define RC_LOST_MS 100
struct RC {
  uint32_t last_rc_tick=1;uint16_t rc_lost_cnt=0;int requested_mode=CONTROL::AUTO,nav_calls=0;
  void ApplyAim();void OnRC();void OnPC();
  void ApplyNav(){++nav_calls;}
  void RC_CheckState(){ctrl.mode=requested_mode;}
  void RC_Control(){
    ctrl.shooter.openRub=false;
    // Preserve the actual RESET branch's Control_Pantile(0,0,0) hold-exit edge.
    if(ctrl.mode==CONTROL::RESET)ctrl.pantile.Control_Pantile(0,0,0);
  }
  bool Shift_mode(){return false;}
};
uint8_t VerifyCRC16CheckSum(uint8_t*,uint32_t);
void AppendCRC16CheckSum(uint8_t*,uint32_t);
