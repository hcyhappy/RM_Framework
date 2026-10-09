# 工程约定

- 本工程用于 STM32F407IGH6 标准 RoboMaster C 板训练；时钟和引脚修改必须核对板型与原理图。
- 板级入口为 `board/`。根 CMake 仅转发到该入口，不维护第二套固件。
- 依赖顺序为 `bsp → libs → modules → threads`；HAL 外设调用集中于 BSP，LED 与 IMU 可按作业要求直接封装 GPIO/SPI。
- CubeMX 生成文件的应用修改放在 USER CODE 区；手写时基和 ThreadX 适配文件不由 CubeMX 维护。
- ThreadX 内核固定为 `v6.4.2_rel`。不要直接修改 `board/Middlewares/ThreadX` 的第三方代码；升级时记录版本和来源。
- CMake 将生成文件中的 PendSV、SysTick、TIM6 和 UART 中断处理重命名；实际向量由 ThreadX/板级适配提供。修改时必须检查 ELF 符号和向量表，避免双重所有者。
- TIM6 为 HAL 1 ms 时基；SysTick 为 ThreadX 1 ms tick。线程内等待使用 `tx_thread_sleep` 或同步对象，避免忙等。ISR 不阻塞、不调 `HAL_Delay`。
- 当前仅启用 alive 和 heartbeat 演示，不启动尚未完成的 IMU、电机、加热或 `bsp_Init`。
- 每一步单独编译并上板验证。编译成功与硬件验收分别记录，不把未观察的结果写成成功。
- 使用小范围提交；文档简洁，调试记录放在 `docs/logs.md`。提交中不包含 build、二进制、个人路径或重复工程压缩包。
