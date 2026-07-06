#include "zf_common_headfile.h"

#pragma location = 0x28001020
__no_init tof_share_t tof_share;

float FlyControl_height = 1.0f;
static float hight = 0.0f;
static float height_dot = 0.0f;
static float last_height = 0.0f;
static uint8_t tof_error_count = 0;
static volatile uint8_t tof_need_reset = 0;
static uint8_t tof_state = 0;

#define TOF_STATE_FIRST_SYNC   (1u << 0)
#define TOF_STATE_NEW_SAMPLE   (1u << 1)

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	
    debug_init();                      
    Attitude_Init();
    dl1b_init();
    motor_init();
    
    // 【新增】初始化 UP-FLOW-302 光流传感器
    upflow302_receive_init();
    printf("UP-FLOW-302 optical flow sensor initialized.\r\n");
    
    pit_ms_init(PIT_CH0, 1);

    while(true)
    {
        if(tof_need_reset)
        {
            tof_need_reset = 0;
            dl1b_init();
            tof_state &= (uint8_t)~TOF_STATE_FIRST_SYNC;
            tof_error_count = 0;
            tof_state &= (uint8_t)~TOF_STATE_NEW_SAMPLE;
            height_dot = 0.0f;
            last_height = 0.0f;
        }
       
        set_pwm();
       
    }
}

void SensorDataGet()
{
    static uint8_t tof_read_count = 0;
    if(++tof_read_count < 20)
    {
        return;
    }

    tof_read_count = 0;
    dl1b_get_distance();

    if(dl1b_distance_mm == 8192)
    {
        tof_state &= (uint8_t)~TOF_STATE_NEW_SAMPLE;
        if(tof_error_count < 255) tof_error_count++;

        if(tof_error_count > 30)
        {
            tof_need_reset = 1;
        }
    }
    else
    {
        tof_state |= TOF_STATE_NEW_SAMPLE;
        tof_error_count = 0;
        hight = (float)dl1b_distance_mm / 1000.0f;

        if((tof_state & TOF_STATE_FIRST_SYNC) == 0)
        {
            height_dot = 0.0f;
            last_height = hight;
            tof_state |= TOF_STATE_FIRST_SYNC;
        }
        else
        {
            height_dot = (hight - last_height) * 50.0f;
            if(height_dot >  3.0f) height_dot =  3.0f;
            if(height_dot < -3.0f) height_dot = -3.0f;
            last_height = hight;
        }
    }

    tof_share.tof_height  = hight;
    tof_share.tof_dist_mm = (int32_t)dl1b_distance_mm;
    SCB_CleanInvalidateDCache_by_Addr(&tof_share, sizeof(tof_share));
}

void pit0_ch0_isr()                     // 锟斤拷时锟斤拷通锟斤拷 0 锟斤拷锟斤拷锟叫断凤拷锟斤拷锟斤拷      
{
    pit_isr_flag_clear(PIT_CH0);
    Attitude_Update();
    SensorDataGet();
    
    // 【新增】光流数据读取和打印 (阶段1验证)
    static uint16_t flow_print_counter = 0;
    if(++flow_print_counter >= 500)  // 每500ms打印一次 (1ms中断 × 500 = 500ms)
    {
        flow_print_counter = 0;
        
        // 打印光流数据用于验证
        printf("[FLOW] State:%d, Valid:%d, X:%d, Y:%d, Time:%d\r\n", 
               upflow302_state_flag,
               upflow302_receive.upflow302_valid,
               upflow302_receive.upflow302_x,
               upflow302_receive.upflow302_y,
               upflow302_receive.upflow302_us);
    }

    PID_Update(&RollPID, 0, SystemIMU.angle.roll + 2.783f);
    PID_Update(&PitchPID, 0, SystemIMU.angle.pitch + 3.17f);

    static uint8_t yaw_locked = 0;
    static float target_yaw = 0.0f;
    float g_yaw_err;
    if(!yaw_locked)
    {
        target_yaw = SystemIMU.angle.yaw;
        yaw_locked = 1;
    }

    g_yaw_err = target_yaw - SystemIMU.angle.yaw;
    while(g_yaw_err >  180.0f) g_yaw_err -= 360.0f;
    while(g_yaw_err < -180.0f) g_yaw_err += 360.0f;
    PID_Update(&YawPID, g_yaw_err, 0.0f);
    PID_Update(&RollRatePID,    RollPID.output,   SystemIMU.gyro_deg[0]);
    PID_Update(&PitchRatePID,   PitchPID.output,  SystemIMU.gyro_deg[1]); 
    PID_Update(&YawRatePID,     YawPID.output ,   SystemIMU.gyro_deg[2]);
    if((tof_state & TOF_STATE_NEW_SAMPLE) != 0)
    {
        tof_state &= (uint8_t)~TOF_STATE_NEW_SAMPLE;
        PID_Update(&HeightPID, FlyControl_height, hight);
        PID_Update(&HeightSpeedPID, HeightPID.output, height_dot);
    }


    
}

// **************************** 锟斤拷锟斤拷锟斤拷锟斤拷 ****************************
