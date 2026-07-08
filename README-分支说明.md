# 项目分支说明

## 当前分支结构

```
main (生产稳定版)
  └── dev (旧开发分支，串口可能损坏) ⚠️ 暂停使用
  └── dev-stable (新稳定开发分支) ⭐ 当前推荐
```

---

## 分支对比

| 特性 | dev | dev-stable |
|------|-----|------------|
| 基准版本 | 多次AI修改累积 | drone-flight-controller-dev(4) |
| 串口通信 | ❌ 可能损坏 | ✅ 已验证稳定 |
| PID参数 | ✅ 最新（三智能体审查优化） | ✅ 最新（相同） |
| 测试状态 | 同事反馈容易报错 | 待测试 |
| 推荐使用 | ❌ | ✅ |

---

## 同事使用指南

### 快速切换
\`\`\`bash
git checkout dev-stable
git pull origin dev-stable
\`\`\`

### 详细说明
见：`切换到dev-stable分支说明.md`

---

## 开发策略调整

### 之前的问题
每次AI修改代码后，串口通信容易出问题，同事调试困难。

### 新的策略
1. **dev-stable 作为稳定基线**
   - 基于已验证可工作的版本
   - 串口通信代码**锁定不改**
   - 只修改算法相关代码（PID参数、控制逻辑等）

2. **代码修改限制**
   - ✅ 可以改：PID.c 参数、main_cm7_0.c 中的控制逻辑
   - ❌ 不要改：wireless_uart_*、cm7_0_isr.c、串口相关初始化

3. **测试流程**
   - 本地修改 → IAR编译验证 → 推送到 dev-stable
   - 同事拉取 → 测试 → 反馈
   - 如果失败，回退单次提交而非重建整个分支

---

## 后续计划

1. ✅ **Phase 1**: dev-stable 分支创建（已完成）
2. ⏳ **Phase 2**: 同事测试 TC03-FINAL
3. ⏳ **Phase 3**: 根据测试结果微调参数
4. ⏳ **Phase 4**: 测试通过后，dev-stable 合并到 main

---

## Git 工作流

### 开发者（AI辅助修改）
\`\`\`bash
git checkout dev-stable
# 修改代码（仅 PID.c 或控制逻辑）
git add <修改的文件>
git commit -m "描述"
git push origin dev-stable
\`\`\`

### 测试者（同事）
\`\`\`bash
git checkout dev-stable
git pull origin dev-stable
# IAR: Clean + Rebuild
# 测试并记录结果
\`\`\`

---

## 紧急回退

如果某次提交导致问题：
\`\`\`bash
git log --oneline -5              # 查看最近5次提交
git revert <commit-hash>          # 回退某次提交
git push origin dev-stable
\`\`\`

或完全回到上一个稳定版本：
\`\`\`bash
git reset --hard <stable-commit>
git push -f origin dev-stable     # 慎用！会覆盖远程
\`\`\`
