#include "Filter.h"

Bessel_Filter_t gyro_lpf[3];        // 三轴角速度贝塞尔滤波器句柄
Bessel_Filter_t acc_lpf[3];         // 三轴角加速度贝塞尔滤波器句柄


/**
 * @brief 初始化二阶贝塞尔低通滤波器
 * @param filter   滤波器结构体指针
 * @param f_sample 采样频率 (比如你的 AHRS_DT 是 1ms，这里就是 1000.0f)
 * @param f_cutoff 截止频率 (比如 35.0f 或 25.0f)
 */
void Filter_Bessel_Init(Bessel_Filter_t *filter, float f_sample, float f_cutoff)
{
    // 贝塞尔滤波器的固有 Q 值 (1 / 根号3)
    float Q = 0.57735027f; 
    
    // 角频率映射
    float omega_0 = 2.0f * M_PI * f_cutoff / f_sample;
    
    // 预计算中间变量
    float cos_omega = cosf(omega_0);
    float alpha = sinf(omega_0) / (2.0f * Q);
    
    // 归一化系数 a0
    float a0 = 1.0f + alpha;

    // 计算标准 Biquad 系数并直接归一化
    filter->b0 = ((1.0f - cos_omega) / 2.0f) / a0;
    filter->b1 =  (1.0f - cos_omega)         / a0;
    filter->b2 = ((1.0f - cos_omega) / 2.0f) / a0;
    filter->a1 = (-2.0f * cos_omega)         / a0;
    filter->a2 =  (1.0f - alpha)             / a0;

    // 清零历史状态
    filter->x1 = 0.0f; filter->x2 = 0.0f;
    filter->y1 = 0.0f; filter->y2 = 0.0f;
    filter->out = 0.0f;
}

/**
 * @brief 应用贝塞尔滤波器 (需在定频中断中调用)
 * @param filter  滤波器结构体指针
 * @param input   当前原始采样值
 * @return 滤波后的平滑值
 */
float Filter_Bessel_Apply(Bessel_Filter_t *filter, float input)
{
    // 标准差分方程: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    filter->out = filter->b0 * input + 
                  filter->b1 * filter->x1 + 
                  filter->b2 * filter->x2 - 
                  filter->a1 * filter->y1 - 
                  filter->a2 * filter->y2;

    // 移位更新历史状态
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = filter->out;

    return filter->out;
}

