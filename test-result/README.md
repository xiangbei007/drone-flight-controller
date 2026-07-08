# test-result 测试结果目录

## 目录说明

同事在执行测试后，将结果文件保存到此目录。

## 命名规范

```
TC03-Phase0-坐标系验证.txt        ← Phase 0 串口输出
TC03-Phase1-贴地悬停.txt          ← Phase 1 串口输出
TC03-Phase1-飞行表现.txt          ← Phase 1 主观观察
TC03-Phase2-50cm系留.txt          ← Phase 2 串口输出
TC03-Phase2-飞行表现.txt          ← Phase 2 主观观察
TC03-Phase3-自由悬停.txt          ← Phase 3 串口输出
TC03-Phase4-抗扰测试.txt          ← Phase 4 串口输出
```

## 提交方式

测试完成后：
```bash
cd ~/code/project/flycontrol
git add test-result/
git commit -m "test: TC03 Phase X 测试结果"
git push origin dev-stable
```
