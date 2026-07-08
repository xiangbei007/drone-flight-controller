# ⚠️ 重要通知：切换到稳定分支 dev-stable

## 问题说明

之前的 `dev` 分支经过多次AI修改，**串口通信代码可能被改坏**，导致同事测试时容易报错。

## 解决方案

创建了新的 **`dev-stable`** 分支，基于已验证的稳定版本（对应 phase2-output-3 测试结果）：

### 保留的稳定代码
- ✅ `wireless_uart_init()` 串口初始化
- ✅ `wireless_uart_send_string()` 无线串口发送
- ✅ `cm7_0_isr.c` 中断处理程序
- ✅ 所有串口/无线通信相关代码

### 仅修改的内容
- ✅ PID参数（Ki=0.03, IntLimit=280, 等）
- ✅ 条件积分抗饱和逻辑
- ✅ trim配平策略（从硬编码改为积分补偿）

---

## 同事操作步骤

### 1. 切换到稳定分支
```bash
cd ~/code/project/flycontrol  # 或你的项目路径
git fetch origin
git checkout dev-stable
git pull origin dev-stable
```

### 2. 验证分支
```bash
git branch  # 应该显示 * dev-stable
git log --oneline -1  # 应该显示: fix: rebase on stable tested version
```

### 3. IAR编译
```
IAR → Project → Clean
IAR → Project → Rebuild All
```

### 4. 验证启动打印

上电后串口应该输出：
```
=== Flight Controller Configuration ===
Trim: Roll=0.0, Pitch=0.0 deg (CG offset compensated by flow integral)
FlowVelXPID: Kp=0.150, Ki=0.0300, Kd=0.080, Limit=10.0, IntLimit=280.0
FlowVelYPID: Kp=0.150, Ki=0.0300, Kd=0.080, Limit=10.0, IntLimit=280.0
Flow control: ENABLED
UP-FLOW-302 optical flow sensor initialized.
========================================
```

**检查点：**
- Ki=0.0300 ✓（不是0.0010）
- IntLimit=280.0 ✓（不是3.0）
- Limit=10.0 ✓（不是4.0）

### 5. 测试流程

按照 `test-case/TC03-FINAL-光流积分补偿测试.md` 执行：

```
Phase 0: 手持坐标系验证（5分钟）← 必须先做！
Phase 1: 10cm贴地悬停（5分钟）
Phase 2: 50cm系留悬停（10分钟）
Phase 3: 1m自由悬停（10分钟）
Phase 4: 抗扰测试（10分钟）
```

---

## 分支对比

| 分支 | 基准版本 | 串口代码 | PID参数 | 状态 |
|------|---------|---------|---------|------|
| **dev-stable** ⭐ | drone-flight-controller-dev(4) | ✅ 稳定 | ✅ 最新优化 | **推荐使用** |
| dev | 多次AI修改 | ❌ 可能损坏 | ✅ 最新优化 | 暂停使用 |

---

## FAQ

### Q: 为什么不修复 dev 分支？
A: dev 分支历史复杂，不确定哪次提交改坏了串口。dev-stable 从已知可工作的版本重新开始，更可靠。

### Q: 如果 dev-stable 测试成功，后续怎么办？
A: dev-stable 会成为新的主开发分支，旧的 dev 可以归档。

### Q: Phase 0 坐标系验证是什么？
A: 手持飞机倾斜10°，观察积分方向。如果方向错误，1-2秒内会失控。**必须先做**。

### Q: 如果串口还是有问题怎么办？
A: 立即反馈，提供：
   - IAR编译输出（有无警告）
   - 串口连接方式（USB/无线）
   - 具体报错信息或现象

---

## 技术细节

### 从旧版本恢复的文件
```
project/user/main_cm7_0.c       ← 从 drone-flight-controller-dev(4) 恢复
project/user/cm7_0_isr.c        ← 从 drone-flight-controller-dev(4) 恢复
```

### 修改的参数
```c
// PID.c (line 254-287)
FlowVelXPID.ki = 0.03f;              // 从 0.001 提升
FlowVelXPID.LimitIntegralMax = 280;  // 从 3 提升
FlowVelXPID.LimitOutputMax = 10.0f;  // 从 4 提升

// main_cm7_0.c (line 197-198)
PID_Update(&RollPID, roll_target, SystemIMU.angle.roll);      // 移除 +2.783
PID_Update(&PitchPID, pitch_target, SystemIMU.angle.pitch);   // 移除 +3.17
```

---

## 联系

如有问题，提供以下信息：
1. 当前分支（`git branch`）
2. 最新提交（`git log -1 --oneline`）
3. 启动打印截图
4. 具体测试阶段和现象
