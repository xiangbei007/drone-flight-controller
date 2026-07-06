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

// 光流速度控制相关变量
static float flow_vel_x = 0.0f;        // 滤波后的光流X速度
static float flow_vel_y = 0.0f;        // 滤波后的光流Y速度
static uint8_t flow_valid = 0;         // 光流数据有效标志
static uint8_t flow_ctrl_enable = 1;   // 光流控制使能（1=开启，0=关闭）

#define FLOW_LPF_ALPHA  0.4f           // 光流低通滤波系数（越小越平滑，越大响应越快）

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
    
    // ====== 光流数据处理 ======
    // 每次中断检查光流是否有新数据（upflow302_finsh_flag由UART中断置位）
    if(upflow302_finsh_flag)
    {
        upflow302_finsh_flag = 0;
        
        if(upflow302_receive.upflow302_valid == 245)
        {
            // 数据有效：一阶低通滤波平滑光流速度
            flow_vel_x = flow_vel_x * (1.0f - FLOW_LPF_ALPHA) + (float)upflow302_receive.upflow302_x * FLOW_LPF_ALPHA;
            flow_vel_y = flow_vel_y * (1.0f - FLOW_LPF_ALPHA) + (float)upflow302_receive.upflow302_y * FLOW_LPF_ALPHA;
            flow_valid = 1;
        }
        else
        {
            // 数据无效：逐渐衰减速度到0（防止突变）
            flow_vel_x *= 0.95f;
            flow_vel_y *= 0.95f;
            flow_valid = 0;
        }
    }
    
    // ====== 光流速度PID控制 ======
    // 目标速度为0（悬停），输出为目标倾角
    float roll_target = 0.0f;
    float pitch_target = 0.0f;
    
    if(flow_ctrl_enable && flow_valid)
    {
        // 光流速度环：速度误差 → 目标角度
        // X轴：光流X正 = 飞机右移 → 需要roll负（左倾）来纠正
        PID_Update(&FlowVelXPID, 0.0f, flow_vel_x);
        roll_target = FlowVelXPID.output;
        
        // Y轴：光流Y正 = 飞机前移 → 需要pitch正（后仰）来纠正
        PID_Update(&FlowVelYPID, 0.0f, flow_vel_y);
        pitch_target = -FlowVelYPID.output;
    }
    
    // ====== 姿态控制 ======
    // 角度环：目标角度(来自光流) → 角速度目标
    PID_Update(&RollPID, roll_target, SystemIMU.angle.roll + 2.783f);
    PID_Update(&PitchPID, pitch_target, SystemIMU.angle.pitch + 3.17f);

    // 偏航锁定
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
    
    // 角速度环
    PID_Update(&RollRatePID,    RollPID.output,   SystemIMU.gyro_deg[0]);
    PID_Update(&PitchRatePID,   PitchPID.output,  SystemIMU.gyro_deg[1]); 
    PID_Update(&YawRatePID,     YawPID.output ,   SystemIMU.gyro_deg[2]);
    
    // 高度环
    if((tof_state & TOF_STATE_NEW_SAMPLE) != 0)
    {
        tof_state &= (uint8_t)~TOF_STATE_NEW_SAMPLE;
        PID_Update(&HeightPID, FlyControl_height, hight);
        PID_Update(&HeightSpeedPID, HeightPID.output, height_dot);
    }

    // ====== 调试输出 ======
    static uint16_t flow_print_counter = 0;
    if(++flow_print_counter >= 500)
    {
        flow_print_counter = 0;
        printf("[FLOW] V:%d Vx:%.1f Vy:%.1f Rt:%.2f Pt:%.2f\r\n",
               flow_valid,
               flow_vel_x, flow_vel_y,
               roll_target, pitch_target);
    }
}

// **************************** 锟斤拷锟斤拷锟斤拷锟斤拷 ****************************
