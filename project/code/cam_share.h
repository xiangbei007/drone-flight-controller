#ifndef CODE_APP_CAM_SHARE_H_
#define CODE_APP_CAM_SHARE_H_

#include <stdint.h>

// ───────────────────────────────────────────────
// 打印/调试模式选择（CM7_0 与 CM7_1 必须取同一值，否则双核争用 UART_1）
//   PRINT_MODE_PARAM : 无线 UART_1 由 CM7_0 发参数遥测（现状保留，方便切回）
//                      CM7_1 静默，只计算 cam_share 供飞控用，不发图也不打印
//   PRINT_MODE_IMAGE : CM7_1 接管无线 UART_1 发整张图像给逐飞助手，
//                      同时用有线 DEBUG_UART(UART_0) printf 打印坐标数据；
//                      此模式下 CM7_0 的无线遥测必须关闭（同一物理 UART 不能双核同写）
//   PRINT_MODE_COORD : CM7_1 接管无线 UART_1 只发坐标(cam_x,cam_y,yaw,valid)，不发图；
//                      避开整张图 ~2s/帧的卡顿，能无线看坐标；
//                      此模式下 CM7_0 的无线遥测同样必须关闭
// ───────────────────────────────────────────────
#define PRINT_MODE_PARAM   (0)
#define PRINT_MODE_IMAGE   (1)
#define PRINT_MODE_COORD   (2)

#define CAM_PRINT_MODE     PRINT_MODE_COORD   // ← 切换打印模式改这里

// 双核共享数据结构（飞控核 CM7_0 读 / 摄像头核 CM7_1 写）
// 必须放置在 0x28001000 共享 SRAM 起点，两核结构体定义须严格一致
typedef struct
{
    int32_t  cam_x;        // V 质心 X (像素，图像坐标 0~MT9V03X_W)
    int32_t  cam_y;        // V 质心 Y (像素，图像坐标 0~MT9V03X_H)
    float    cam_yaw_rad;  // V 输出方向 (弧度，指向 V 尖端)
    uint32_t cam_valid;    // 0 = 本帧未检测到，1 = 检测到

    // 逐飞助手在线调参（CM7_1 收无线调参包后写 / CM7_0 读并应用到姿态补偿）
    float    roll_trim;    // roll 安装补偿 (度)，默认 2.783
    float    pitch_trim;   // pitch 安装补偿 (度)，默认 3.17
    uint32_t trim_seq;     // 每次收到新调参 +1，CM7_0 据此判断有无更新
} cam_share_t;             // 共占 28 字节，落在 0x28001000 这一条 cache line(32B) 内

// ───────────────────────────────────────────────
// TOF 高度共享数据（飞控核 CM7_0 写 / 摄像头核 CM7_1 读）
//   方向与 cam_share 相反，故必须放在【独立的 cache line】上，
//   否则两核各自的 CleanInvalidateDCache 会互相覆盖对方写入的字段。
//   cam_share 占 0x28001000~0x2800101F（一条 32B line），本结构放下一条 line 起点 0x28001020。
// ───────────────────────────────────────────────
typedef struct
{
    float    tof_height;   // 滤波后高度 (m)
    int32_t  tof_dist_mm;  // 原始测距 (mm)，8192=错误/超量程
} tof_share_t;

#endif
