#include "pid.h"
#include "math.h"

#define PID_DT_SECONDS (0.001f)

//清除积累量,用于停止控制时使用
void PID_Reset(PIDControllerType_t* pid)
{
	pid->intergral = 0;
	pid->output = 0;
	pid->prev_error = 0;
	pid->derivative = 0;
}

//PID更新函数
void PID_Update(PIDControllerType_t* pid,float target,float current)
{
	
	float error = target - current; //本次误差
	
	//积分因子
	float integral_factor = (fabs(error) < pid->IntegralThreshold) ? 1.0f : (pid->IntegralThreshold / fabs(error));
	pid->intergral += integral_factor  * error;
	

	//微分项,采用一阶低通滤波
	float Tmpderivative = (error - pid->prev_error);
	pid->derivative = pid->alpha*Tmpderivative + (1.0f-pid->alpha)*pid->derivative;
	pid->prev_error = error;
	
	//pid输出
	pid->output = pid->kp * error + pid->ki * pid->intergral + pid->kd*pid->derivative;
	
	//输出限幅
	if( pid->output > pid->LimitOutputMax ) 
	{
		pid->output = pid->LimitOutputMax;
		if( error>0 ) pid->intergral *= 0.9f; // 轻微减少积分项，防止累积过多
	}
	if( pid->output < pid->LimitOutputMin ) 
	{
		pid->output = pid->LimitOutputMin;
		if( error<0 ) pid->intergral *= 0.9f; // 轻微减少积分项，防止累积过多
	}
	
	//积分限幅
	if( pid->intergral>pid->LimitIntegralMax ) pid->intergral = pid->LimitIntegralMax;
	if( pid->intergral<pid->LimitIntegralMin ) pid->intergral = pid->LimitIntegralMin;
	
}

//角度环roll
PIDControllerType_t RollPID = {
	.kp = 5.0f,    // 增加到 5.0，提供足够的角速度指令
	.ki = 0.003f,  // 折中值（原0.001太小，0.005有低频震荡风险）
	.kd = 0.3f,
	.LimitIntegralMax = 30.0f,
	.LimitIntegralMin = -30.0f,    // 积分限幅,30度
	.LimitOutputMax = 150.0f,      // 角速度限幅,限制 ~100度/s (约1.745 rad/s) 更稳定
	.LimitOutputMin = -150.0f,
	.IntegralThreshold = 8.0f,    // 折中值（原10太保守，5太激进）
	.alpha = 0.9f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角速度环roll
PIDControllerType_t RollRatePID = {
	.kp = 6.0f,    // 小幅降低到 6.0
	.ki = 0.02f,
	.kd = 0.0f,
	.LimitIntegralMax = 3000.0f,  // 折中（原4000，降到3000保留恢复能力）
	.LimitIntegralMin = -3000.0f,
	.LimitOutputMax = 3000.0f,    // 折中（2000太保守会恢复不了，3000兼顾安全和能力）
	.LimitOutputMin = -3000.0f,
	.IntegralThreshold =20.0f ,   // 20度/s开始积分
	.alpha = 0.8f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角度环pitch
PIDControllerType_t PitchPID = {
	.kp = 5.0f,    // 增加到 5.0，提供足够的角速度指令
	.ki = 0.003f,  // 折中值（原0.001太小，0.005有低频震荡风险）
	.kd = 0.5f,
	.LimitIntegralMax = 30.0f,
	.LimitIntegralMin = -30.0f,    // 积分限幅,30度
	.LimitOutputMax = 150.0f,      // 角速度限幅,限制 ~100度/s (约1.745 rad/s) 更稳定
	.LimitOutputMin = -150.0f,
	.IntegralThreshold = 8.0f,    // 折中值（原10太保守，5太激进）
	.alpha = 0.9f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角速度环pitch
PIDControllerType_t PitchRatePID = {
	.kp = 6.0f,   // 增加到 10.0，提供足够的油门修正
	.ki = 0.05f,
	.kd = 0.8f,
	.LimitIntegralMax = 3000.0f,  // 折中（原4000，降到3000保留恢复能力）
	.LimitIntegralMin = -3000.0f,
	.LimitOutputMax = 3000.0f,    // 折中（2000太保守，3000兼顾安全和能力）
	.LimitOutputMin = -3000.0f,
	.IntegralThreshold = 20.0f ,  // 20度/s开始积分
	.alpha = 0.8f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角度环yaw
PIDControllerType_t YawPID = {
	.kp = 2.8f,
	.ki = 0.002f,
	.kd = 0.2f,
	.LimitIntegralMax = 30.0f,
	.LimitIntegralMin = -30.0f,
	.LimitOutputMax = 500.0f,      // 与 Roll/Pitch 一致，输出为角速度目标(度/s)
	.LimitOutputMin = -500.0f,
	.IntegralThreshold = 10.0f ,  // 10度误差有效积分
	.alpha = 1.0f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

//角速度环yaw
PIDControllerType_t YawRatePID = {
	.kp = 1.6f,
	.ki = 0.06f,
	.kd = 0.0f,
	.LimitIntegralMax = 60.0f,   // 积分限幅,60度/s
	.LimitIntegralMin = -60.0f,
	.LimitOutputMax = 4000,
	.LimitOutputMin = -4000,
	.IntegralThreshold = 10.0f ,
        .alpha = 0.8f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

// 高度环
PIDControllerType_t HeightPID = {
	.kp = 6.927f,
	.ki = 0.0013f,
	.kd = 0.0f,
	.LimitIntegralMax = 10000.0f,
	.LimitIntegralMin = -10000.0f,
	.LimitOutputMax = 90000.0f,    // 爬升：保持不限幅，保证起飞推力（误差1m≈+1714油门）
	.LimitOutputMin = -90000.0f,      // 下降：限制下降指令，防止积分负向饱和把电机压到怠速
	.IntegralThreshold = 10 ,
	.alpha = 1.0f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

// 高度速度环
PIDControllerType_t HeightSpeedPID = {
	.kp = 233.0f,//295.0f
	.ki = 0.0f,//0.45f
	.kd = 0.0f,
	.LimitIntegralMax = 90000.0f,
	.LimitIntegralMin = -90000.0f,
	.LimitOutputMax = 90000.0f,
	.LimitOutputMin = -90000.0f,
	.IntegralThreshold = 20 ,
	.alpha = 0.4,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};


// ============================================================
// 视觉伺服 —— 摄像头位置保持 PID（外环）
// 输入：图案中心与屏幕中心的像素误差 (px)
// 输出：目标倾角 (°)    传给 RollPID / PitchPID
// ============================================================
// 调参指南：
//   kp 大 → 飞机朝图案移动更快，但太大晃
//   ki 大 → 消除中心点固定偏移（静差）
//   kd 大 → 到位后减少来回摆
//   LimitOutput → 限制飞机最大倾角，越大移动越快
//   IntegralThreshold → 误差小于此值才积分（避免积分饱和）
// ============================================================

// ---- X轴（左右）位置保持 ----
PIDControllerType_t CamPosPIDX = {
	.kp = 0.778f,        // 像素→倾角(°)，图案偏1px飞机倾0.5°
	.ki = 0.0f,      // 积分：消除静差，让图案精确回中
	.kd = 0.022f,       // 微分：防止来回震荡
	.LimitIntegralMax = 2.0f,    // 积分上限2°
	.LimitIntegralMin = -2.0f,
	.LimitOutputMax =  5.0f,     // 倾角保护：目标 roll 最大 +5°
	.LimitOutputMin = -5.0f,     // 目标 roll 最小 -5°
	.IntegralThreshold = 5.0,    // 误差<5px才积分，避免大误差饱和
	.alpha = 0.8f,               // 微分低通滤波
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

// ---- Y轴（前后）位置保持 ----
// 注意：PitchPID有+5.87°配平，LimitOutput必须大于配平值才能后仰
PIDControllerType_t CamPosPIDY = {
	.kp = 0.8827f,        // 像素→倾角(°)
	.ki = 0.0f,      // 积分：消除静差
	.kd = 0.012f,       // 微分：防震荡
	.LimitIntegralMax = 3.0f,
	.LimitIntegralMin = -3.0f,
	.LimitOutputMax = 5.0f,         // 倾角保护：目标 pitch 最大 +5°（>配平3.17°，仍能后仰）
	.LimitOutputMin = -5.0f,        // 目标 pitch 最小 -5°
	.IntegralThreshold = 5.0,
	.alpha = 0.8f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

// ============================================================
// 光流速度控制 PID（外环）
// 输入：滤波后的光流速度 (像素/帧)
// 输出：目标倾角 (°)    传给 RollPID / PitchPID
// ============================================================
// 控制链路：
//   光流速度(目标0) → FlowVelPID → 目标角度 → 角度PID → 角速度PID → 电机
//
// 调参指南：
//   kp 大 → 抗飘能力强，但容易震荡
//   ki 大 → 消除持续漂移（风/配平不准），但积分饱和风险
//   kd 大 → 减速时更快稳住，但放大噪声
//   LimitOutput → 限制飞机最大倾角，防止光流反馈导致过度倾斜
// ============================================================

// ---- X轴（左右）速度控制 ----
// 光流X正方向 = 飞机向右移动，需要向左倾斜(roll负)来纠正
PIDControllerType_t FlowVelXPID = {
	.kp = 0.15f,         // 降低增益，防止过度响应（从0.3降到0.15）
	.ki = 0.001f,        // 消除持续漂移
	.kd = 0.05f,         // 抑制速度震荡
	.LimitIntegralMax = 3.0f,
	.LimitIntegralMin = -3.0f,
	.LimitOutputMax =  4.0f,     // 降低最大倾角限制到±4°（从±8°降低）
	.LimitOutputMin = -4.0f,
	.IntegralThreshold = 10.0f,  // 速度误差<10时积分
	.alpha = 0.6f,               // 微分低通滤波
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};

// ---- Y轴（前后）速度控制 ----
// 光流Y正方向 = 飞机向前移动，需要向后倾斜(pitch正)来纠正
PIDControllerType_t FlowVelYPID = {
	.kp = 0.15f,         // 降低增益（从0.3降到0.15）
	.ki = 0.001f,        // 消除持续漂移
	.kd = 0.05f,         // 抑制速度震荡
	.LimitIntegralMax = 3.0f,
	.LimitIntegralMin = -3.0f,
	.LimitOutputMax =  4.0f,     // 降低最大倾角限制到±4°
	.LimitOutputMin = -4.0f,
	.IntegralThreshold = 10.0f,
	.alpha = 0.6f,
	.prev_error = 0,
	.intergral = 0,
	.derivative = 0,
	.output = 0
};