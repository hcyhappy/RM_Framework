# ThreadX 接入与本机操作

## 本次修改

基于仓库 `ba86c11`：CMake 启用 C11/C++17/ASM，加入 BSP、libs、modules、threads 源码；内核固定为 Eclipse ThreadX `v6.4.2_rel`，Cortex-M4 GNU 端口，来源见 `board/Middlewares/ThreadX/VERSION.md`。

- `board/Core/Inc/tx_user.h`：1000 tick/s，32 级优先级，启用栈检查。
- `board/Core/Src/threadx_port.c`：初始化 SysTick、向量表与调度优先级，提供 SysTick/UART 中断。
- `board/Core/Src/hal_timebase.c`：TIM6 为独立 HAL 毫秒时基，HAL_Init 和时钟切换时自动重配。
- `threads/src/app_threadx.cpp`：调用 tx_kernel_enter，在 tx_application_define 中分配内存、创建信号量及线程。
- `threads/src/alivethread.cpp`：等待演示心跳，通过绿色 LED 反映存活，超时亮红灯。
- `main.c`：只在 USER CODE 区调用应用入口。
- 修正 UART NVIC，使 USART1/6 和 DMA 中断可以配合工作。后续 BSP 接入已启动双串口 DMA 接收，详见 docs/bsp.md。
- 将原 BMI088 中未配置的 htim10 引用置于默认关闭的加热开关，允许框架编译；未补完 IMU、PID 或电机功能。

## 启动顺序与时基

启动文件初始化数据、BSS 和 C++ 静态对象 → main → HAL_Init（TIM6 开始计时）→ SystemClock_Config（TIM6 随时钟重配）→ 外设初始化 → bsp_Init → app_threadx_start → tx_kernel_enter → _tx_initialize_low_level → tx_application_define → ThreadX 调度。

HAL 的 uwTick 来自 TIM6，每毫秒增加一次；ThreadX 的 tx_time_get 来自 SysTick，每毫秒一个 tick。现在 tx_thread_sleep(200) 是约 200 ms。同步测试不要在 ISR 或 tx_application_define 中调用 HAL_Delay；内核初始化期间全局中断关闭。

ThreadX 官方 Cortex-M4 GNU 汇编提供 PendSV_Handler。CubeMX 的同名空函数若直接链接会冲突，因此 CMake 仅对生成的 stm32f4xx_it.c 重命名对应函数。TIM6、UART 也由板级手写适配统一维护，CubeMX 生成的副本不占用向量。

## 内存与线程

| 内存/线程 | 大小或优先级 | 用途 |
|---|---|---|
| C 库 heap | 8192 B | 链接脚本预留；当前线程创建不使用 malloc |
| MSP stack | 8192 B | 启动、异常、中断使用 |
| ThreadX byte pool | 静态 8192 B | 独立于 C 库 heap，包含分配管理开销 |
| alive 栈 | 从 pool 分配 1024 B | 等待心跳并更新灯 |
| heartbeat demo 栈 | 从 pool 分配 1024 B | 每 200 ms 发送测试信号量 |
| ThreadX timer thread 栈 | 内核静态 1024 B | 内核软件定时器 |
| alive 优先级 | 15 | 较低优先级 |
| heartbeat demo 优先级 | 10 | 比 alive 更高 |

ThreadX 优先级数字越小越高。byte pool 剩余空间不是额外 RAM，只是 8192 B 中尚未分配的部分。启用线程栈检查并安装报错回调；`app_init_status` 检查初始化结果，`app_stack_error` 指示检测到栈错误。栈检查不替代运行中的栈余量测量。

IMU 和电机线程没有创建；demo 心跳不代表这两项通过监控验收。后续扩展需要增加各自信号量与超时判定。

## 应用补丁

补丁适用于 `ba86c11` 基线。在 VS Code 工程根终端：

```bash
git status
git switch -c feat/threadx-bringup
git apply --whitespace=nowarn --check threadx-bringup.patch
git apply --whitespace=nowarn threadx-bringup.patch
```

先保存并提交自己的现有修改，再检查应用补丁。若 `--check` 报错，不要强行覆盖，提供当前改动再适配。补丁包含内核源码，不需要另行下载软件包或初始化子模块。

## 构建

本机需安装 Arm GNU Toolchain（arm-none-eabi-gcc/g++）、CMake >=3.22、Ninja，并加入 PATH。在 VS Code 终端验证：

```bash
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

建议从 `board/` 构建：

```bash
cd board
cmake --preset Debug
cmake --build --preset Debug
```

生成 `board/build/Debug/board.elf`、`board.hex`、`board.bin`、`board.map`。根目录也能使用同名 preset；生成目录为根 `build/Debug`，固件位于其 `board/` 子目录。工具链路径或工程目录变化后请删除旧 build 目录再配置。

关闭演示生产线程、仅验证 LED sleep：

```bash
cmake --preset Debug -DRM_ENABLE_HEARTBEAT_DEMO=OFF
cmake --build --preset Debug
```

默认 `RM_ENABLE_IMU_HEATER=OFF`；开启前必须配置 TIM10、提供 htim10 并实现加热控制，不能仅打开开关。

## CubeMX 操作

本次手工接入，不需要在 CubeMX 中安装或启用 ThreadX 软件包。

1. 打开 `board/board.ioc`，System Core → SYS → Timebase Source 确认 TIM6。
2. TIM6 专供 HAL 时基，不在应用层再次调用 MX_TIM6_Init，也不用于 PWM。
3. NVIC：SysTick 优先级 4、PendSV/SVC 15；TIM6 15；UART/CAN/DMA 6。SysTick/PendSV 不生成空处理函数，运行时低层适配会再次设置核心异常优先级。
4. Generate Code 后重新运行 CMake 配置。`main.c` 的 USER CODE 调用必须保留。
5. CMake 排除 CubeMX 的 stm32f4xx_hal_timebase_tim.c，采用手写 hal_timebase.c；没有两个 HAL_InitTick。
6. 不要把手写时基逻辑搬回 SysTick_Handler。确认 build map 中实际 SysTick 来自 threadx_port.c，PendSV 来自内核汇编，TIM6 来自 hal_timebase.c。

生成后的 main.c 可能包含默认 TIM6 周期回调；实际手写 TIM6 IRQ 直接调用 HAL_IncTick，不经过该回调，保证只加一次。将来需要 TIM6 的其他功能时先重设计时基。

## 上板验收

下载自己构建的 board.elf/hex，复位后：

- 绿色 LED 每约 200 ms 切换一次，完整亮灭周期约 400 ms。
- 正常情况下红灯、蓝灯保持关闭。
- 调试器观察 app_init_status=TX_SUCCESS，app_stack_error=0；app_heartbeat_count 与 app_alive_count 持续增加。
- 暂停/挂起 heartbeat demo 线程后，alive 在约 1 s 超时，红灯亮；恢复生产线程后红灯熄灭、绿灯恢复切换。不要暂停整个 CPU 来测试超时。
- 对比相同墙钟时间内 HAL_GetTick 和 tx_time_get 的增量；两者均约每秒 1000。HAL 时基在内核启动前也要能工作。

编译与 ELF 检查不能证明硬件调度成功，需要用户实际下载后记录现象。
