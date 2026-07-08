# test-case 测试用例目录

## 当前唯一需要执行的测试

### ⭐ TC03-FINAL — 光流积分补偿测试

**文件**: `TC03-FINAL-光流积分补偿测试.md`

按 Phase 0 → Phase 1 → Phase 2 → Phase 3 → Phase 4 顺序执行，任何阶段失败不得跳级。

---

## 硬件要求

- 系留绳 **2米**（不是50cm）
- 地面有明显纹理
- 光照充足

## 测试前必须

1. `git checkout dev-stable && git pull origin dev-stable`
2. IAR: **Clean + Rebuild All**
3. 准备2米系留绳

## 测试流程

```
Phase 0: 手持坐标系验证（5分钟）← 不可跳过！
Phase 1: 10cm贴地悬停（5分钟）
Phase 2: 50cm系留悬停（10分钟）
Phase 3: 1m自由悬停（10分钟）
Phase 4: 抗扰测试（10分钟）← 阶段2通过标准
```
