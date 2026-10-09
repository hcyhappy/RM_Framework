
## 2026-10-09：构建与 ThreadX 接入

- 基线：ba86c11。固定 ThreadX v6.4.2_rel，加入 Cortex-M4 GNU 端口。
- CMake 加入框架 C++ 源码，统一 board/ 入口；HAL 时基改为 TIM6，ThreadX 使用 SysTick。
- 创建 byte pool、心跳信号量、alive 与 heartbeat demo，检查创建返回值并启用栈检查。
- Arm GNU 13.2.1、CMake 3.28.3、Ninja 1.11.1：板级 Debug/Release 和根入口 Debug（关闭心跳演示）均构建成功，完整 Release/根构建日志未见编译警告。
- 默认 Debug：FLASH 33908 B；常规 SRAM 27784 B（含 8 KiB heap、8 KiB MSP 和静态线程池），CCM 0 B。Release：FLASH 18708 B。
- 检查 ELF 向量：PendSV、SysTick、TIM6、USART1、USART6 均指向预期实现；链接脚本 heap/stack 预留确认为各 8192 B。
- 未执行 CubeMX 实际重新生成，也未上板；线程切换、LED 灯效、tick 增量和故障监控均待硬件验证。
