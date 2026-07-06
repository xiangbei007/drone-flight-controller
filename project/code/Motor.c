#include "Motor.h"

MOTOR_t motor;
uint32 PID_BaseSpeed = 4800;  // 折中值（原4200过低，5200上行余量不足）

void motor_init(void)
{
    pwm_init(PWM_upper_left , FREQ, 0);
    pwm_init(PWM_upper_right, FREQ, 0);
    pwm_init(PWM_lower_left , FREQ, 0);
    pwm_init(PWM_lower_right, FREQ, 0);
}

uint32 target_limit_uint32(uint32 insert, uint32 low, uint32 high)
{
    if      (insert <  low) return low;
    else if (insert > high) return high;
    else return insert;
}

uint32 throttle_to_duty(uint32 throttle)
{
    if (throttle <= Throttle_MIN)  return PWM_MIN;
    if (throttle >= Throttle_MAX)  return PWM_MAX;
    return PWM_MIN + (uint32)((throttle - Throttle_MIN) * 500/*(PWM_MAX - PWM_MIN)*/ / (Throttle_MAX - Throttle_MIN));
}

void set_pwm(void)
{
    // 使用 int32_t 计算防止负数溢出（致命bug修复）
    int32_t thr_a = (int32_t)PID_BaseSpeed - (int32_t)RollRatePID.output + (int32_t)PitchRatePID.output + (int32_t)YawRatePID.output + (int32_t)HeightSpeedPID.output;
    int32_t thr_b = (int32_t)PID_BaseSpeed + (int32_t)RollRatePID.output + (int32_t)PitchRatePID.output - (int32_t)YawRatePID.output + (int32_t)HeightSpeedPID.output;
    int32_t thr_c = (int32_t)PID_BaseSpeed - (int32_t)RollRatePID.output - (int32_t)PitchRatePID.output - (int32_t)YawRatePID.output + (int32_t)HeightSpeedPID.output;
    int32_t thr_d = (int32_t)PID_BaseSpeed + (int32_t)RollRatePID.output - (int32_t)PitchRatePID.output + (int32_t)YawRatePID.output + (int32_t)HeightSpeedPID.output;

    // 负值保护后限幅
    motor.A.throttle = target_limit_uint32((uint32)(thr_a < (int32_t)Throttle_MIN ? Throttle_MIN : thr_a), Throttle_MIN, Throttle_MAX);
    motor.B.throttle = target_limit_uint32((uint32)(thr_b < (int32_t)Throttle_MIN ? Throttle_MIN : thr_b), Throttle_MIN, Throttle_MAX);
    motor.C.throttle = target_limit_uint32((uint32)(thr_c < (int32_t)Throttle_MIN ? Throttle_MIN : thr_c), Throttle_MIN, Throttle_MAX);
    motor.D.throttle = target_limit_uint32((uint32)(thr_d < (int32_t)Throttle_MIN ? Throttle_MIN : thr_d), Throttle_MIN, Throttle_MAX);
	
    pwm_set_duty(PWM_lower_right, throttle_to_duty(motor.A.throttle));
    pwm_set_duty(PWM_lower_left, throttle_to_duty(motor.B.throttle));
    pwm_set_duty(PWM_upper_right, throttle_to_duty(motor.C.throttle));
    pwm_set_duty(PWM_upper_left, throttle_to_duty(motor.D.throttle));

}