#include "Attitude_Est.h"

/* ===========================================全局变量定义===============================================*/
Attitude_Handle_t SystemIMU;            // 姿态解算全局句柄，外部通过此结构体获取最终角度

/* =========================================内部静态函数声明===============================================*/
static float InvSqrt(float x);
static void IMU_Read_And_Preprocess(Attitude_Handle_t *imu);
static void Mahony_Update(Attitude_Handle_t *imu);
static void Post_Processing_Filter(Attitude_Handle_t *imu);

/**
 * @description: 初始化姿态解算系统
 * @return {*}
 * @note: 1. 包含硬件初始化检查
 *        2. 执行陀螺仪零偏静态校准 + Mahony 收敛 + 地球系加速度残余采集
 *        3. pitch/roll 不做抹零，启动时机身相对真实重力倾多少度，输出就是多少度
 *        4. 注意：初始化过程中请保持静止，不要移动传感器
 * @example:
 */
void Attitude_Init(void)
{
    Attitude_Handle_t *imu = &SystemIMU;

    /*1. 初始化 IMU 硬件*/ 
    if(IMU_INIT())
    {
        // 初始化失败处理，死循环卡住 (实际使用可改为错误灯提示)
        zf_log(0, "IMU Init Failed!");
        while(1);
    }

    /*2. 初始化四元数 (初始状态为水平)*/
    imu->q0 = 1.0f;
    imu->q1 = 0.0f;
    imu->q2 = 0.0f;
    imu->q3 = 0.0f;

    /*3. 清零积分误差*/
    imu->exInt = 0; imu->eyInt = 0; imu->ezInt = 0;

    /*4. 陀螺仪零偏校准 & 噪声评估*/
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    /*5. 用于记录最大值和最小值 (初始化为极大/极小)*/
    float gz_min = 100000.0f;
    float gz_max = -100000.0f;

    float gyro_scale = IMU_GYRO_SCALE;          // 获取驱动中的陀螺仪比例系数

    /*6. 陀螺仪零漂校准*/
    for(int i = 0; i < GYRO_CALIB_COUNT; i++)
    {
        IMU_GET_GYRO();     // 读取底层数据 (使用抽象宏)

        // 转换到底层单位 (建议统一转为 rad/s 进行计算，这里沿用你的逻辑先处理)
        float current_gz = (float)IMU_GYRO_Z / gyro_scale;

        // 累加 (临时转换为 deg/s 方便观察，后续统一转 rad/s)
        gx_sum += (float)IMU_GYRO_X / gyro_scale;
        gy_sum += (float)IMU_GYRO_Y / gyro_scale;
        gz_sum += (float)IMU_GYRO_Z / gyro_scale;

        // 记录波动的最大值和最小值
        if(current_gz > gz_max) gz_max = current_gz;
        if(current_gz < gz_min) gz_min = current_gz;

        system_delay_ms(2); // 延时等待下一次采样
    }

    // 计算平均值并转换为 rad/s 存储到全局变量
    imu->gyro_bias[0] = (gx_sum / GYRO_CALIB_COUNT) * DTOR;
    imu->gyro_bias[1] = (gy_sum / GYRO_CALIB_COUNT) * DTOR;
    imu->gyro_bias[2] = (gz_sum / GYRO_CALIB_COUNT) * DTOR;

    // 打印陀螺仪零偏校准结果（单位：度/秒）
    printf("Gyro Bias: X=%.2f, Y=%.2f, Z=%.2f deg/s\r\n",
           imu->gyro_bias[0] * RTOD, imu->gyro_bias[1] * RTOD, imu->gyro_bias[2] * RTOD);

    /* 7. 计算动态死区阈值*/
    // 噪声峰峰值 = max - min
    // 理论上偏差幅度是峰峰值的一半，但为了保险，我们乘一个安全系数 (比如 1.2 或 1.5)
    // 这样能把 99.9% 的静止噪声都过滤掉
    float noise_amplitude_z = (gz_max - gz_min); 
    
    // 阈值 = (噪声幅度 / 2) * 安全系数(1.5) * 单位转换
    imu->gyro_deadzone[2] = ((noise_amplitude_z / 2.0f) * 3.0f) * DTOR;
    if(imu->gyro_deadzone[2] > (0.5f * DTOR))
    {
        imu->gyro_deadzone[2] = 0.5f * DTOR;
    }

    /*8. 初始化二阶贝塞尔滤波器*/
    // 三轴角速度
    Filter_Bessel_Init(&gyro_lpf[0], 1.0f / AHRS_DT, GYRO_CUTOFF_FREQ); 
    Filter_Bessel_Init(&gyro_lpf[1], 1.0f / AHRS_DT, GYRO_CUTOFF_FREQ);
    Filter_Bessel_Init(&gyro_lpf[2], 1.0f / AHRS_DT, GYRO_CUTOFF_FREQ);
    // 三轴加速度
    Filter_Bessel_Init(&acc_lpf[0], 1.0f / AHRS_DT, ACC_CUTOFF_FREQ);
    Filter_Bessel_Init(&acc_lpf[1], 1.0f / AHRS_DT, ACC_CUTOFF_FREQ);
    Filter_Bessel_Init(&acc_lpf[2], 1.0f / AHRS_DT, ACC_CUTOFF_FREQ);
    
    /*9. Mahony 收敛 + acc_earth 静态残余采集*/
    // 设计意图：
    //   - pitch/roll 不做任何抹零：开机时机身相对真实重力倾 5°，输出就报 5°。
    //     Mahony 算法本身就是把四元数对齐"真实地心重力"的，让它跑 2 秒收敛即可。
    //   - 只采集 acc_earth 的静态残余 (acc_bias)：用于高度环融合时把"静止悬浮"的
    //     微小残差归零，这与角度零点无关，是位置/速度积分链路的独立需求。
    for(int i = 0; i < 2000; i++)
    {
        Attitude_Update();
        system_delay_ms(1);
    }

    float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
    for(int i = 0; i < 1000; i++)
    {
        Attitude_Update();

        // 把当前机体加速度投影到地球系
        float ax = imu->acc_g[0], ay = imu->acc_g[1], az = imu->acc_g[2];
        float q0 = imu->q0, q1 = imu->q1, q2 = imu->q2, q3 = imu->q3;
        float a_earth_x_g = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*ax + 2.0f*(q1*q2 - q0*q3)*ay + 2.0f*(q1*q3 + q0*q2)*az;
        float a_earth_y_g = 2.0f*(q1*q2 + q0*q3)*ax + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*ay + 2.0f*(q2*q3 - q0*q1)*az;
        float a_earth_z_g = 2.0f*(q1*q3 - q0*q2)*ax + 2.0f*(q2*q3 + q0*q1)*ay + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*az;

        ax_sum += a_earth_x_g;
        ay_sum += a_earth_y_g;
        az_sum += a_earth_z_g;

        system_delay_ms(1);
    }

    // 静止时地球系 X/Y 理论上为 0，Z 理论上为 1.0g。残差留作 acc_earth 净加速度的零偏补偿。
    imu->acc_bias[0] = ax_sum / 1000.0f;
    imu->acc_bias[1] = ay_sum / 1000.0f;
    imu->acc_bias[2] = az_sum / 1000.0f;

    printf("Acc Earth Bias: X=%.4f Y=%.4f Z=%.4f g\r\n",
           imu->acc_bias[0], imu->acc_bias[1], imu->acc_bias[2]);
}


/**
 * @description: 姿态解算核心任务 (周期性调用)
 * @return {*}
 * @note: 必须在定时中断中调用，周期需与 AHRS_DT (默认1ms) 保持一致
 *        流程：读取数据 -> Mahony算法更新四元数 -> 欧拉角转换与滤波
 * @example: 
 */
void Attitude_Update(void)
{
    Attitude_Handle_t *imu = &SystemIMU;

    /*1. 读取硬件数据，转物理单位，低通滤波*/ 
    IMU_Read_And_Preprocess(imu);

    /*2. 对输出的三轴角速度信息执行二阶滤波 */
    Filter_Bessel_Apply(&gyro_lpf[0], SystemIMU.gyro_deg[0]);
    Filter_Bessel_Apply(&gyro_lpf[1], SystemIMU.gyro_deg[1]);
    Filter_Bessel_Apply(&gyro_lpf[2], SystemIMU.gyro_deg[2]);

    /*3. 用滤波后的°/s角速度单位覆盖掉原来的rad/s单位，传入Mahony解算*/
    imu->gyro_rad[0] = gyro_lpf[0].out * DTOR;
    imu->gyro_rad[1] = gyro_lpf[1].out * DTOR;
    imu->gyro_rad[2] = gyro_lpf[2].out * DTOR;

    /*4. 使用动态计算的阈值进行判断死区*/ 
    if(fabs(imu->gyro_rad[2]) <= imu->gyro_deadzone[2])
    {
        imu->gyro_rad[2] = 0.0f; // 在噪声范围内，认为是静止
    }
    else
    {
        // 超过噪声范围，认为是有效运动
        // 优化写法：为了平滑，可以减去死区值（可选，防止突变）
        if(imu->gyro_rad[2] > 0) 
        {
            imu->gyro_rad[2] = imu->gyro_rad[2] - imu->gyro_deadzone[2];
        }
        else               
        {
            imu->gyro_rad[2] = imu->gyro_rad[2] + imu->gyro_deadzone[2];
        }
    }
    /*5. Mahony 互补滤波解算*/
    Mahony_Update(imu);

    /*6. 坐标系调整与输出平滑*/
    Post_Processing_Filter(imu);
}

/*====================================内部静态函数 (Static Functions)==============================================*/
/**
 * @description: 读取传感器并预处理
 * @param {Attitude_Handle_t} *imu姿态句柄指针
 * @return {*}
 * @note: 内部调用
 *        1. 将寄存器原始整数转换为物理单位 (g, rad/s)
 *        2. 对加速度计数据进行一阶低通滤波 (系数 alpha = 0.3)
 *        3. 对陀螺仪数据去除零偏
 * @example: 
 */
static void IMU_Read_And_Preprocess(Attitude_Handle_t *imu)
{
    // --- 1. 获取原始数据 ---
    IMU_GET_ACC();
    IMU_GET_GYRO();

    // --- 2. 获取转换系数 (自动适配驱动中的量程设置) ---
    float acc_scale  = IMU_ACC_SCALE;
    float gyro_scale = IMU_GYRO_SCALE;

    float cur_ax = (float)IMU_ACC_X / acc_scale;
    float cur_ay = (float)IMU_ACC_Y / acc_scale;
    float cur_az = (float)IMU_ACC_Z / acc_scale;

    // --- 3. 将原始数据喂入二阶滤波器进行更新
    Filter_Bessel_Apply(&acc_lpf[0], cur_ax);
    Filter_Bessel_Apply(&acc_lpf[1], cur_ay);
    Filter_Bessel_Apply(&acc_lpf[2], cur_az);

    // 取出滤波后的平滑数据存入姿态句柄
    // (这里假设你的滤波器结构体中，输出值存放在 .out 成员中，与你陀螺仪的用法保持一致)
    imu->acc_g[0] = acc_lpf[0].out;
    imu->acc_g[1] = acc_lpf[1].out;
    imu->acc_g[2] = acc_lpf[2].out;

    // --- 4. 处理陀螺仪 (灵敏度转换（+单位转换） + 去除零偏) ---

    // 计算公式：(原始值 / 灵敏度比例系数 * PI/180) - 静态零偏————————单位rad/s
    imu->gyro_rad[0] = ((float)IMU_GYRO_X / gyro_scale) * DTOR - imu->gyro_bias[0];
    imu->gyro_rad[1] = ((float)IMU_GYRO_Y / gyro_scale) * DTOR - imu->gyro_bias[1];
    imu->gyro_rad[2] = ((float)IMU_GYRO_Z / gyro_scale) * DTOR - imu->gyro_bias[2];

    // 计算公式：(原始值 / 灵敏度比例系数) - 静态零偏————————————————单位°/s
    imu->gyro_deg[0] = imu->gyro_rad[0] * RTOD;
    imu->gyro_deg[1] = imu->gyro_rad[1] * RTOD;
    imu->gyro_deg[2] = imu->gyro_rad[2] * RTOD;
}

/**
 * @description: Mahony 互补滤波算法核心
 * @param {Attitude_Handle_t} *imu：姿态句柄指针
 * @return {*}
 * @note: 内部调用
 *        1. 利用重力加速度修正陀螺仪积分误差 (PI控制器)
 *        2. 使用一阶龙格库塔法(Runge-Kutta)更新四元数
 * @example: 
 */
static void Mahony_Update(Attitude_Handle_t *imu)
{
    float ax = imu->acc_g[0];
    float ay = imu->acc_g[1];
    float az = imu->acc_g[2];
    float gx = imu->gyro_rad[0];
    float gy = imu->gyro_rad[1];
    float gz = imu->gyro_rad[2];

    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 1. 加速度归一化 (只利用方向，忽略大小)
    norm = InvSqrt(ax*ax + ay*ay + az*az);
    if(norm == 0.0f) return;
    ax *= norm; ay *= norm; az *= norm;

    // 2. 估计重力方向 (将当前姿态下的重力向量转到机体坐标系)
    float q0 = imu->q0, q1 = imu->q1, q2 = imu->q2, q3 = imu->q3;
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3. 计算误差 (测量重力与估计重力的叉积)
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = 0.0f; // 仅使用加速度计进行滚转俯仰校正，忽略偏航校正

    // 4. PI 控制器修正 (比例+积分)
    imu->exInt += ex * AHRS_KI * AHRS_DT;
    imu->eyInt += ey * AHRS_KI * AHRS_DT;
    // imu->ezInt += ez * AHRS_KI * AHRS_DT;

    gx += AHRS_KP*ex + imu->exInt;
    gy += AHRS_KP*ey + imu->eyInt;
    gz += AHRS_KP*ez + imu->ezInt;

    // 5. 四元数微分方程更新
    float halfT = 0.5f * AHRS_DT;
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;

    imu->q0 += (-q1_last*gx - q2_last*gy - q3_last*gz) * halfT;
    imu->q1 += ( q0_last*gx + q2_last*gz - q3_last*gy) * halfT;
    imu->q2 += ( q0_last*gy - q1_last*gz + q3_last*gx) * halfT;
    imu->q3 += ( q0_last*gz + q1_last*gy - q2_last*gx) * halfT;

    // 6. 四元数归一化 (保持单位模长)
    norm = InvSqrt(imu->q0*imu->q0 + imu->q1*imu->q1 + imu->q2*imu->q2 + imu->q3*imu->q3);
    imu->q0 *= norm; imu->q1 *= norm; imu->q2 *= norm; imu->q3 *= norm;
}

/**
 * @description: 后处理与滤波
 * @param {Attitude_Handle_t} *imu：姿态句柄指针
 * @return {*}
 * @note: 内部调用
 *        1. 四元数 -> 欧拉角 (Z-Y-X 顺序)
 *        2. 输出值平滑滤波 (50%新 + 50%旧)
 *        3. 计算连续的 Total Yaw (处理 ±180 度跳变)
 * @example: 
 */
static void Post_Processing_Filter(Attitude_Handle_t *imu)
{
    float temp_roll, temp_pitch, temp_yaw;

    // 1. 四元数转欧拉角 (标准航空次序 Z-Y-X)
    temp_pitch = asinf(-2*imu->q1*imu->q3 + 2*imu->q0*imu->q2) * RTOD;
    temp_roll  = atan2f(2*imu->q2*imu->q3 + 2*imu->q0*imu->q1, -2*imu->q1*imu->q1 - 2*imu->q2*imu->q2 + 1) * RTOD;
    temp_yaw   = atan2f(2*imu->q1*imu->q2 + 2*imu->q0*imu->q3, -2*imu->q2*imu->q2 - 2*imu->q3*imu->q3 + 1) * RTOD;

    imu->angle.roll  = temp_roll;
    imu->angle.pitch = temp_pitch;

    // 2. Yaw 累计处理 (连续化)
    float yaw_diff = temp_yaw - imu->last_yaw;
    // 处理过零跳变 (如 +179 -> -179)
    if(yaw_diff < -180.0f) yaw_diff += 360.0f;
    if(yaw_diff >  180.0f) yaw_diff -= 360.0f;

    // Apply axis correction for upright installation (X and Z inverted)
    imu->angle.yaw = temp_yaw;
    imu->angle.total_yaw += yaw_diff;
    imu->last_yaw = temp_yaw;

    // 3. [新增] 计算地球坐标系下的 Z 轴净加速度 (用于高度环融合)
    // 取出经过低通滤波的机体加速度 (单位: g)
    float ax = imu->acc_g[0];
    float ay = imu->acc_g[1];
    float az = imu->acc_g[2];
    
    // 取出当前最新的四元数
    float q0 = imu->q0, q1 = imu->q1, q2 = imu->q2, q3 = imu->q3;

    // 使用四元数旋转矩阵的第三行，将机体加速度投影到地球坐标系 Z 轴 (竖直向上)
    float a_earth_x_g = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*ax + 2.0f*(q1*q2 - q0*q3)*ay + 2.0f*(q1*q3 + q0*q2)*az;
    float a_earth_y_g = 2.0f*(q1*q2 + q0*q3)*ax + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*ay + 2.0f*(q2*q3 - q0*q1)*az;
    float a_earth_z_g = 2.0f * (q1 * q3 - q0 * q2) * ax + 2.0f * (q2 * q3 + q0 * q1) * ay + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * az;

    // 【核心修复】：统一减去各个轴的静态残余偏置，并转为 mm/s^2
    float raw_acc_x_earth = (a_earth_x_g - imu->acc_bias[0]) * 9806.65f;
    float raw_acc_y_earth = (a_earth_y_g - imu->acc_bias[1]) * 9806.65f;
    float raw_acc_z_earth = (a_earth_z_g - imu->acc_bias[2]) * 9806.65f;
    
    
    // 【新增】：计算 X 和 Y 轴净加速度 (转为 mm/s^2)
    // 注意：水平方向本来就没有重力分量，所以不需要减去 1G (bias)
    // 如果想要波形好看，也可以给它们加上轻度低通
    static float lpf_acc_x_earth = 0.0f;
    static float lpf_acc_y_earth = 0.0f;
    static float lpf_acc_z_earth = 0.0f;

    lpf_acc_x_earth = lpf_acc_x_earth * 0.85f + raw_acc_x_earth * 0.15f;
    lpf_acc_y_earth = lpf_acc_y_earth * 0.85f + raw_acc_y_earth * 0.15f;
    lpf_acc_z_earth = lpf_acc_z_earth * 0.85f + raw_acc_z_earth * 0.15f;
    
    imu->acc_earth[0] = lpf_acc_x_earth;
    imu->acc_earth[1] = lpf_acc_y_earth;   
    imu->acc_earth[2] = lpf_acc_z_earth;
}

/**
 * @description: 快速平方根倒数算法 (Fast Inverse Square Root)
 * @param {float} x：输入数值
 * @return {*float}：1/sqrt(x)
 * @note: 内部调用，经典的 Quake III 算法，比系统库函数更快
 * @example: 
 */
static float InvSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
