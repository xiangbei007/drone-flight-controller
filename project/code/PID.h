#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "pid.h"

typedef struct{
	float kp; // 比例增益
	float ki; // 积分增益
	float kd; // 微分增益
	float prev_error; // 上一次误差
	float intergral;  // 积分项
	float derivative; //微分项
	float output;     // 输出
	float LimitIntegralMax; //积分上限
	float LimitIntegralMin; //积分下限
	float LimitOutputMax;   //输出上限
	float LimitOutputMin;   //输出下限
	float IntegralThreshold;//积分阈值
	float alpha;            //一阶低通滤波系数
}PIDControllerType_t;

void PID_Update(PIDControllerType_t* pid,float target,float current);
void PID_Reset(PIDControllerType_t* pid);

extern PIDControllerType_t RollRatePID;
extern PIDControllerType_t PitchRatePID;
extern PIDControllerType_t RollPID;
extern PIDControllerType_t PitchPID;
extern PIDControllerType_t YawRatePID;
extern PIDControllerType_t YawPID;
extern PIDControllerType_t HeightSpeedPID;
extern PIDControllerType_t HeightPID;
extern PIDControllerType_t CamPosPIDY;
extern PIDControllerType_t CamPosPIDX;

#endif