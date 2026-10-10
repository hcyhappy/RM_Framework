
## 2026-10-09：构建与 ThreadX 接入

- 基线：ba86c11。固定 ThreadX v6.4.2_rel，加入 Cortex-M4 GNU 端口。
- CMake 加入框架 C++ 源码，统一 board/ 入口；HAL 时基改为 TIM6，ThreadX 使用 SysTick。
- 创建 byte pool、心跳信号量、alive 与 heartbeat demo，检查创建返回值并启用栈检查。
- Arm GNU 13.2.1、CMake 3.28.3、Ninja 1.11.1：板级 Debug/Release 和根入口 Debug（关闭心跳演示）均构建成功，完整 Release/根构建日志未见编译警告。
- 默认 Debug：FLASH 33908 B；常规 SRAM 27784 B（含 8 KiB heap、8 KiB MSP 和静态线程池），CCM 0 B。Release：FLASH 18708 B。
- 检查 ELF 向量：PendSV、SysTick、TIM6、USART1、USART6 均指向预期实现；链接脚本 heap/stack 预留确认为各 8192 B。
- 未执行 CubeMX 实际重新生成，也未上板；线程切换、LED 灯效、tick 增量和故障监控均待硬件验证。

## 2026-10-09：BSP CAN/UART/PWM

- 基线：main 57bb6a19b3b6e63bf3de68fb0c115a1e7c136189，已合并 ThreadX。
- 补全带返回状态的 CAN/UART/PWM 封装；MX 初始化后检查 bsp_Init，再进入内核。CAN 无自动电流命令，PWM 不自动启动。
- CAN 过滤器在两次配置前均设置 SlaveStartFilterBank=14；增加标准帧校验、16帧/总线接收队列、邮箱与 bus-off 检查。
- 同步 can.c 的 AutoBusOff/AutoRetransmission 为 ENABLE，与原有 ioc 一致。
- UART 验证句柄、DMA 映射和 NVIC；复制异步 TX 数据；Normal DMA idle/TC 复制到511字节队列后重装填。错误恢复由 USART_Service 在线程中执行，发送异常时保留缓冲区直到 DMA 中止完成。
- PWM 支持 TIM1 CH1~4、TIM8 CH1~3；校验周期、占空比与脉宽，禁止运行时改共享周期。
- LED GPIO 调用集中到 modules/src/LED.cpp；线程不直接调用外设 HAL。
- Arm GNU 13.2.1：board Debug/Release、根 Debug（heartbeat demo OFF）构建成功；最终三个构建日志未见 warning/error。
- 默认 Debug FLASH 40020 B，普通 SRAM 30472 B（含预留），CCM 0 B；Release FLASH 21856 B；根 Debug FLASH 38876 B、SRAM 30288 B。
- 直接读取 ELF 向量表，核对 PendSV、SysTick、TIM6、CAN1/2 RX0、USART1/6、DMA2 Stream1/2/6/7 共11个入口，均匹配实际符号。
- Linux x86-64 HAL 模拟测试（UBSan）通过：未初始化、错误参数/映射/NVIC、CAN队列、UART复制/重装填/溢出/错误恢复、PWM边界和多通道、PRIMASK恢复。
- 按用户要求跳过硬件测试。未运行 CubeMX GUI 重新生成；Normal DMA 连续流量丢失窗口、电气与真实并发时序仍需上板验证。

## 2026-10-10：libs PID 与基础函数

- 基线：main 474cef570bb2502bb4fff265851e86fbdfaa1219，已包含BSP。
- 补全位置式与增量式PID，输出限幅与累计积分限幅；增量式使用实际允许的积分增量，GetIntegralOutput用于观察累计I。
- Clear、模式变更与非法输入清理历史，Tuning拒绝非有限参数；double中间量避免限幅前溢出，尚未测量MCU执行时间。
- 保留CRC表与正常报文结果、CRC16字节顺序和最短长度约定；补齐CRC8空指针保护，CRC16空指针返回调用方种子。
- Numeric::LimitABS补充负/非有限上限与NaN输入处理。
- g++主机测试通过（ASan/UBSan；LeakSanitizer因沙箱进程访问限制关闭，不作为泄漏验证）：公式、饱和/反向积分、1万次限幅、Clear/Tuning/模式切换、异常浮点、独立逐位CRC参考及损坏报文。
- Arm GNU13.2.1板级Debug/Release构建通过，日志未见warning/error；Debug FLASH40020 B、Release FLASH21856 B，普通SRAM均30472 B。PID尚未接入运行控制链路，未引用函数可被链接器移除。
- 未上板、不启动电机或加热；闭环参数整定、实际采样周期和运行耗时待硬件验证。

## 2026-10-10：module 传感器与 M2006

- 基线：main 31eec5219a9cf0f3f0e3d957a5d659c6e55ff445，已合并 PID。
- BMI088 使用生成的 SPI1/PA4/PB0：读协议统一丢弃 dummy，有限超时、CS 释放、ID 和配置有效位回读；±6g m/s²、±2000°/s 换算 rad/s、11位温度和无效值判断。陀螺仪带宽 bit7 忽略；去除他板预标定偏置。
- M2006/C610：总线/ID 注册，CAN 队列轮询和可选其他报文分发，大端打包、反馈/回绕/36:1 输出轴单位，速度环和位置速度双环；默认 PID 尚未整定，不自动启动。离线/队列丢帧放松并取消位置参考，ZeroPosition 显式建立参考。C610 不解析温度，电流反馈保留手册原始单位。
- 新增 IST8310：I²C1/PG6，硬复位和 ID，默认单次测量、至少5ms间隔、DRDY轮询、小端 XYZ ×0.3 µT、重触发和错误返回；STAT2 bit3 不误当溢出。HAL I²C 和毫秒时钟集中于新 BSP 包装。
- LED 先前已实现，核对 PH10/11/12 低有效并加入模拟测试。
- g++ + UBSan 的模块协议/控制测试通过；已有 BSP 和 libs 测试回归通过。
- Arm GNU13.2.1板级 Debug/Release 编译通过；默认 FLASH40020/21856 B、SRAM30472 B。额外强制链接30个模块/I²C入口成功，FLASH58532 B、SRAM30872 B，无未解析符号；此为链接测试，不运行新增模块。
- 核对默认 Debug ELF 的11个 RTOS/HAL/CAN/UART/DMA 向量入口，均匹配实际符号。构建日志无warning/error。
- 按用户要求未上板；未改 board.ioc、启动线程、外设中断或第三方内核。不自动初始化传感器、电机驱动或加热；采样时序、PID参数、机械原点、CAN负载和线程栈余量待硬件/threads阶段验证。
