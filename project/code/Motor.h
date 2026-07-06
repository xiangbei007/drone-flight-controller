#ifndef __Motor_H
#define __Motor_H

#include "zf_common_headfile.h"
#include "Motor.h"

#define FREQ      (50)
#define PWM_upper_left     (TCPWM_CH20_P08_1)   // 左前
#define PWM_upper_right    (TCPWM_CH11_P05_2)   // 右前
#define PWM_lower_left     (TCPWM_CH58_P17_3)   // 左后
#define PWM_lower_right    (TCPWM_CH31_P10_3)   // 右后

//油门范围，频率50hz
#define PWM_MIN 500
#define PWM_MAX 1200
#define Throttle_MIN 500
#define Throttle_MAX 7000

typedef struct{
	uint8 Telemetry; //遥测标志位
	uint32 throttle; //油门值
} MotorVal_t;

typedef struct {
	MotorVal_t A;
    MotorVal_t B;
	MotorVal_t C;
	MotorVal_t D;
} MOTOR_t;

// extern MOTOR_t motor;

void motor_init(void);
void set_pwm(void);

#endif