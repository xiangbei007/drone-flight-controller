#ifndef CODE_ATTITUDE_EST_H_
#define CODE_ATTITUDE_EST_H_

#include "zf_common_headfile.h"

/* ==============================================硬件抽象层=================================================*/
#define IMU_TYPE_660RA              1
#define IMU_TYPE_660RB              2
#define IMU_TYPE_963RA              3

#define ACTIVE_IMU  IMU_TYPE_963RA                                  //在此可一键更改使用陀螺仪类型

#if ACTIVE_IMU == IMU_TYPE_660RA
    #define IMU_INIT()             imu660ra_init()
    #define IMU_GET_ACC()          imu660ra_get_acc()
    #define IMU_GET_GYRO()         imu660ra_get_gyro()
    #define IMU_ACC_X              imu660ra_acc_x
    #define IMU_ACC_Y              imu660ra_acc_y
    #define IMU_ACC_Z              imu660ra_acc_z
    #define IMU_GYRO_X             imu660ra_gyro_x
    #define IMU_GYRO_Y             imu660ra_gyro_y
    #define IMU_GYRO_Z             imu660ra_gyro_z
    #define IMU_ACC_SCALE          imu660ra_transition_factor[0]    // 660RA 比例系数从驱动的全局数组获取
    #define IMU_GYRO_SCALE         imu660ra_transition_factor[1]

#elif ACTIVE_IMU == IMU_TYPE_963RA
    #define IMU_INIT()             imu963ra_init()
    #define IMU_GET_ACC()          imu963ra_get_acc()
    #define IMU_GET_GYRO()         imu963ra_get_gyro()
    // Hardware remap: logical X = -raw Y, logical Y = raw X. Gyro Z is inverted so clockwise yaw is positive.
    #define IMU_ACC_X              (-imu963ra_acc_y)
    #define IMU_ACC_Y              ( imu963ra_acc_x)
    #define IMU_ACC_Z              ( imu963ra_acc_z)
    #define IMU_GYRO_X             (-imu963ra_gyro_y)
    #define IMU_GYRO_Y             ( imu963ra_gyro_x)
    #define IMU_GYRO_Z             (-imu963ra_gyro_z)
    #define IMU_ACC_SCALE          imu963ra_transition_factor[0]    // 963RA 比例系数从驱动的全局数组获取
    #define IMU_GYRO_SCALE         imu963ra_transition_factor[1]

#elif ACTIVE_IMU == IMU_TYPE_660RB
    #define IMU_INIT()             imu660rb_init()
    #define IMU_GET_ACC()          imu660rb_get_acc()
    #define IMU_GET_GYRO()         imu660rb_get_gyro()
    #define IMU_ACC_X              (imu660rb_acc_y)
    #define IMU_ACC_Y              (-imu660rb_acc_x)
    #define IMU_ACC_Z              (imu660rb_acc_z)

    #define IMU_GYRO_X             (imu660rb_gyro_y)
    #define IMU_GYRO_Y             (-imu660rb_gyro_x)
    #define IMU_GYRO_Z             (imu660rb_gyro_z)
    // 660RB 的比例系数在逐飞库里没有导出为全局变量，目前按库里的默认配置写死
    // 加速度 ±8G -> 4098.0f，陀螺仪 ±1000dps -> 28.6f
    #define IMU_ACC_SCALE          4098.0f
    #define IMU_GYRO_SCALE         28.6f
    // #define IMU_PITCH_DIR          (1.0f)
#endif

/* =======================================算法参数配置 (可根据实际情况调整)========================================*/
#define AHRS_DT             0.001f          // 姿态解算采样周期 [单位: 秒]
                                            // 务必与定时中断周期保持一致 (当前对应 1ms)

#define AHRS_KP             2.0f           // 互补滤波比例增益 (Proportional Gain)
                                            // 决定加速度计修正陀螺仪的速度。值越大，收敛越快，但对震动越敏感。    加减震：2.0；不加减震：0.75

#define AHRS_KI             0.008f          // 互补滤波积分增益 (Integral Gain)
                                            // 用于消除陀螺仪的稳态漂移。值越大，消除漂移越快，但容易引起低频震荡。

#define GYRO_CALIB_COUNT    2000            // 陀螺仪零偏校准的采样次数
                                            // 启动时读取 n 次求平均值作为零偏。耗时 = 次数 * 间隔(1ms)

#define GYRO_CUTOFF_FREQ    150             // 陀螺仪二阶贝塞尔滤波器截止频率
                                            // 高于这个频率的信号将会被过滤掉，截止频率越高，响应越好，噪声越大     加减震：150；不加减震：100

#define ACC_CUTOFF_FREQ     80            // 角速度计二阶贝塞尔滤波器截止频率
                                            // 高于这个频率的信号将会被过滤掉，截止频率越高，响应越好，噪声越大     加减震：80；不加减震：35

/* =============================================数学常数与转换因子===============================================*/
#ifndef M_PI
#define M_PI              3.1415926535f
#endif

#define RTOD              (57.2957795f) // 弧度转角度系数 (180 / PI)
#define DTOR              (0.01745329f) // 角度转弧度系数 (PI / 180)


/* ===============================================数据结构定义===================================================*/
// 1. 欧拉角结果结构体 (用于控制系统的最终输出)
typedef struct 
{
    float roll;         // 横滚角 [单位: 度 °]
                        // 左右倾斜，向右倾斜通常为正(取决于安装方向)

    float pitch;        // 俯仰角 [单位: 度 °]
                        // 前后倾斜，车头抬起通常为正(取决于安装方向)

    float yaw;          // 偏航角 [单位: 度 °] 范围: -180 ~ +180
                        // 绕Z轴旋转，存在不连续跳变

    float total_yaw;    // 累计偏航角 [单位: 度 °] 范围: 无限制
                        // 连续累加的角度，无跳变，适合用于转弯闭环控制
} EulerAngle_t;

// 2. 姿态解算器全集句柄 (包含状态量、中间量、结果)
typedef struct 
{
    // --- 内部状态量 ---
    float q0, q1, q2, q3;       // 四元数 (Quaternion)，描述当前的姿态估计值
    float exInt, eyInt, ezInt;  // 积分误差累积值 (用于校正陀螺仪漂移)
    float gyro_bias[3];         // 陀螺仪静态零偏 [x, y, z] (单位: rad/s)，初始化时计算得出
    float acc_bias[3];          // 地球系加速度静态残余 (用于 acc_earth 净加速度补偿)
    float gyro_deadzone[3];     // 存储 XYZ 三轴动态计算出的死区阈值

    // --- 历史数据 ---
    float last_roll;            // 上一次计算的横滚角 (用于平滑滤波)
    float last_pitch;           // 上一次计算的俯仰角 (用于平滑滤波)
    float last_yaw;             // 上一次计算的偏航角 (用于处理 Total Yaw 的过零跳变)

    // --- 原始数据缓存 ---
    float acc_g[3];             // 归一化后的加速度数据 [x, y, z] (单位: g)
    float gyro_rad[3];          // 转换后的角速度数据   [x, y, z] (单位: rad/s)
    
    // --- 输出结果 (Output) ---
    EulerAngle_t angle;         // 当前解算出的欧拉角
    float gyro_deg[3];        // 转换后的角速度数据   [x, y, z] (单位: °/s)
    float acc_earth[3];         // 地球系X、Y、Z轴净加速度 (mm/s^2)
} Attitude_Handle_t;


/* ==============================================全局变量声明========================================================*/
extern Attitude_Handle_t SystemIMU;     // 姿态解算系统全局实例
                                        // 外部调用示例: float my_roll = SystemIMU.angle.roll;


/* ================================================函数声明==========================================================*/
/**
 * @description: 姿态解算初始化
 * @return {*}
 * @note: 
 * @example: 
 */
void Attitude_Init(void);


/**
 * @description: 姿态解算mahony更新
 * @return {*}
 * @note: 确保在定时中断中的调用周期和Attitude_Est.h宏定义处相匹配
 * @example: 
 */
void Attitude_Update(void);

#endif