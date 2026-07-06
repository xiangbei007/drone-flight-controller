#ifndef __FILTER_H__
#define __FILTER_H__

#include "zf_common_headfile.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//======================================滤波器句柄类型定义===================================

// 贝塞尔滤波器结构体
typedef struct {
    float b0, b1, b2, a1, a2; // 滤波器系数
    float x1, x2;             // 历史输入
    float y1, y2;             // 历史输出
    float out;                // 当前滤波结果
} Bessel_Filter_t;


//======================================滤波器全局变量声明===================================
extern Bessel_Filter_t gyro_lpf[3];         // 定义三个轴的陀螺仪的二阶贝塞尔滤波器
extern Bessel_Filter_t acc_lpf[3];          // 定义三个轴的加速度计的二阶贝塞尔斯滤波器

//======================================滤波器函数申明=====================================

/**
 * @brief 初始化二阶贝塞尔低通滤波器
 * @param filter   滤波器结构体指针
 * @param f_sample 采样频率 (比如你的 AHRS_DT 是 1ms，这里就是 1000.0f)
 * @param f_cutoff 截止频率
 */
void Filter_Bessel_Init(Bessel_Filter_t* filter, float f_sample, float f_cutoff);

/**
 * @brief 应用贝塞尔滤波器 (需在定频中断中调用)
 * @param filter  滤波器结构体指针
 * @param input   当前原始采样值
 * @return 滤波后的平滑值
 */
float Filter_Bessel_Apply(Bessel_Filter_t* filter, float input);

#endif