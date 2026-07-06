#include "Motor.h"

MOTOR_t motor;
uint32 PID_BaseSpeed = 4200;

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
    motor.A.throttle = (uint32)(PID_BaseSpeed - RollRatePID.output + PitchRatePID.output + YawRatePID.output + HeightSpeedPID.output);
    motor.B.throttle = (uint32)(PID_BaseSpeed + RollRatePID.output + PitchRatePID.output - YawRatePID.output + HeightSpeedPID.output);
    motor.C.throttle = (uint32)(PID_BaseSpeed - RollRatePID.output - PitchRatePID.output - YawRatePID.output + HeightSpeedPID.output);
    motor.D.throttle = (uint32)(PID_BaseSpeed + RollRatePID.output - PitchRatePID.output + YawRatePID.output + HeightSpeedPID.output);

    motor.A.throttle = target_limit_uint32(motor.A.throttle, Throttle_MIN, Throttle_MAX);
    motor.B.throttle = target_limit_uint32(motor.B.throttle, Throttle_MIN, Throttle_MAX);
    motor.C.throttle = target_limit_uint32(motor.C.throttle, Throttle_MIN, Throttle_MAX);
    motor.D.throttle = target_limit_uint32(motor.D.throttle, Throttle_MIN, Throttle_MAX);
	
    pwm_set_duty(PWM_lower_right, throttle_to_duty(motor.A.throttle));
    pwm_set_duty(PWM_lower_left, throttle_to_duty(motor.B.throttle));
    pwm_set_duty(PWM_upper_right, throttle_to_duty(motor.C.throttle));
    pwm_set_duty(PWM_upper_left, throttle_to_duty(motor.D.throttle));

}