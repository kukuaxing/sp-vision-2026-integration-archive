// Execute the real controller against simulated CAN feedback and DBUS packets.
#include "test_stubs/can_mock.h"
#include "../STM32F405/pitch_zero.cpp"
#include <cassert>
#include <cstdio>
CAN can1, can2;
volatile uint32_t calibration_pitch_rx_count=0, calibration_pitch_rx_tick=0;
volatile uint32_t calibration_yaw_rx_count=0, calibration_yaw_rx_tick=0;
volatile uint32_t small_yaw_rx_count=0, small_yaw_rx_tick=0;
int32_t small_command=0;
uint32_t now=1;
bool feedback=true;
float yaw_speed=0;
int pitch_enable=0, yaw_enable=0, yaw_transmits=0;
uint32_t HAL_GetTick() { return now; }
int CAN::Transmit(uint32_t id, const uint8_t* b, uint8_t len) {
    assert(len==8);
    if(this==&can2) ++yaw_transmits;
    assert((this==&can1 && (id==0x109 || id==0x7ff || id==0x1ff)) || (this==&can2 && id==0x206));
    if(id==0x1ff) {
        for(int i=2;i<8;++i) assert(b[i]==0);
        small_command=int16_t(uint16_t(b[0])<<8|b[1]);
        assert(abs(small_command)<=2500); return 0;
    }
    if(id==0x7ff || !feedback) return 0;
    bool p=this==&can1;
    uint8_t* r=DMmotor_data[p?1:0];
    bool special=true; for(int i=0;i<7;++i) special &= b[i]==255;
    if(special) {
        assert(b[7]==0xfc || b[7]==0xfd);
        r[0]=(p?9:6) | (b[7]==0xfc ? 16:0);
        if(b[7]==0xfc) (p?pitch_enable:yaw_enable)++;
    } else if(p) {
        float target; memcpy(&target,b,4);
        int raw=int(lroundf((target+12.56637f)*65535/25.13274f));
        r[1]=raw>>8; r[2]=raw;
    } else memcpy(&yaw_speed,b,4);
    if(p) {++calibration_pitch_rx_count; calibration_pitch_rx_tick=now;}
    else {++calibration_yaw_rx_count; calibration_yaw_rx_tick=now;}
    return 0;
}
void frame(int pitch=0,int yaw=0,int left=1,int right=2,int small=0) {
    uint64_t bits=uint64_t(yaw+1024) | (uint64_t(1024)<<11) | (uint64_t(small+1024)<<22) | (uint64_t(pitch+1024)<<33) | (uint64_t(right)<<44) | (uint64_t(left)<<46);
    uint8_t b[18]{}; for(int i=0;i<6;++i) b[i]=bits>>(i*8);
    GimbalRemoteFrame(b);
}
void step(int n=1,bool radio=true,int pitch=0,int yaw=0,int left=1,int right=2,int small=0) {
    for(int i=0;i<n;++i) {now+=5; ++small_yaw_rx_count; small_yaw_rx_tick=now; if(radio) frame(pitch,yaw,left,right,small); PitchZeroUpdate();}
}
int main(int argc,char**argv) {
    uint16_t start=PITCH_ZERO_RAW+100;
    can1.DMmotor_data[1][1]=start>>8; can1.DMmotor_data[1][2]=start;
    can2.DMmotor_data[0][1]=128;
    if(argc>1 && strcmp(argv[1], "small_wrap")==0) {
        can1.data[4][0]=0x1f; can1.data[4][1]=0xfe;
        small_yaw_rx_count=1; small_yaw_rx_tick=now;
        SmallYawUpdate(true,0,now);
        now+=5; ++small_yaw_rx_count; small_yaw_rx_tick=now;
        can1.data[4][0]=0; can1.data[4][1]=2;
        SmallYawUpdate(true,0,now);
        assert(small_yaw_state==3 && fabsf(small_yaw_relative_rad - 4*6.283185307f/8192)<.00001f);
        small_yaw_rx_tick=now+1; SmallYawUpdate(true,0,now);
        assert(small_yaw_state==3); // ISR can update timestamp after caller snapshots now.
        now+=102; SmallYawUpdate(true,660,now);
        assert(small_yaw_state==4 && small_yaw_fault==1 && small_command==0);
        puts("PASS: small Y encoder wrap and stale-feedback stop"); return 0;
    }
    if(argc>1) {
        start=PITCH_ZERO_RAW+1000;
        can1.DMmotor_data[1][1]=start>>8; can1.DMmotor_data[1][2]=start;
        step(10); assert(pitch_zero_state==4 && pitch_zero_fault==3 && pitch_enable==0);
        puts("PASS: out-of-corridor startup never enables"); return 0;
    }
    step(250,true,660); // Startup ignores a deflected stick, then requires centering.
    assert(pitch_home_ready && !gimbal_remote_active && pitch_enable==1);
    assert(fabsf(pitch_command_rad-PITCH_ZERO_RAD)<.001f);
    step(10); assert(gimbal_remote_active && yaw_manual_state==3);
    assert(small_yaw_state==3);
    step(20,true,0,0,1,2,660); assert(small_command<0);
    step(20,true,0,0,1,2,-660); assert(fabsf(small_yaw_target_rad)<.001f);
    step(1,true,0,0,1,1); assert(small_command==0);
    step(10);
    step(100,true,660,660);
    assert(pitch_command_rad<PITCH_ZERO_RAD-.04f && yaw_speed<0);
    step(600,true,660,0);
    assert(fabsf(pitch_command_rad-(PITCH_ZERO_RAD-.18f))<.001f);
    step(25,false); assert(!gimbal_remote_active && yaw_speed==0 && yaw_manual_state==0);
    float stopped=pitch_command_rad; step(100,false); assert(pitch_command_rad==stopped);
    step(10,true,-660); assert(!gimbal_remote_active); // reconnect requires neutral
    step(10); step(20,true,-660); assert(pitch_command_rad>stopped);
    step(1,true,0,0,1,1); assert(!gimbal_remote_active && yaw_speed==0);
    step(10); step(1,true,900); assert(!gimbal_remote_active && yaw_speed==0);
    step(10); can1.DMmotor_data[1][0]=0xc9;
    ++calibration_pitch_rx_count; step(1);
    assert(pitch_zero_state==4 && pitch_zero_fault==2 && !gimbal_remote_active && yaw_speed==0);
    int enables=pitch_enable; step(100); assert(pitch_enable==enables);
    yaw_manual_state=4; int tx_before=yaw_transmits; step(100);
    assert(yaw_transmits-tx_before<=2);
    small_yaw_state=3; small_yaw_fault=0;
    can1.data[4][0]=0x10; can1.data[4][1]=0;
    step(1); assert(small_yaw_state==4 && small_yaw_fault==3 && small_command==0);
    puts("PASS: small Y output isolation/direction/limits and startup, neutral interlock, both axes, pitch clamp, radio loss/reconnect, mode exit, invalid frame, fault latch");
}
