# BSP：CAN、UART、PWM 接入与学习说明

## 本次基线与范围

基于 `hcyhappy/RM_Framework` 的 `main` 提交 `57bb6a19b3b6e63bf3de68fb0c115a1e7c136189`（已合并 ThreadX）。补全 CAN、UART、PWM 的原有接口，并增加接收队列、统计和舵机微秒脉宽接口。

业务层通过 BSP 访问 CAN/UART/PWM。LED 的 HAL GPIO 操作集中到 `modules/src/LED.cpp`，线程改用 LED 接口；IMU 后续可按作业规定在模块中封装 HAL SPI/GPIO。CubeMX 的初始化和 IRQ 转发仍属于板级代码，不需要强行移进 BSP。

原来 BSP 初始化函数返回 void，现在返回 `HAL_StatusTypeDef`。原有调用若忽略返回值仍可编译，但必须逐步改为检查。UART mode 枚举明确采用 `uint8_t` 底层类型；工程源码需整体重新构建，不沿用旧对象文件。

## 启动顺序与边界

`main()` 先执行全部 `MX_*`，随后检查 `bsp_Init()`，最后进入 ThreadX。BSP 不重新调用 `MX_*`：

1. `PWM_Init()` 验证 TIM1/TIM8 初始化；不启动输出。
2. `USART_Init()` 验证双 UART、DMA 配置和 NVIC，启动内部 Receive-to-Idle Normal DMA 接收。
3. `CAN_Init()` 完整配置过滤器、启动控制器、启用 FIFO0 消息通知。

初始化失败时启动代码点亮红灯并进入 `Error_Handler()`。初始化可能部分成功，例如 UART1 已启动而 UART6 失败；总返回值失败意味着整体启动未完成，不能继续执行业务。

初始化应在中断允许的主程序阶段完成，不放在 ThreadX 的 `tx_application_define()` 中。CAN 启动与某些 HAL abort 使用毫秒超时；全局关中断时 TIM6 无法推进超时。

不会自动发送 CAN 电流帧或启动 PWM。电机控制模块、PID、IMU 和裁判协议不在本次实现范围。

## 返回值约定

| 返回值 | 含义 | 上层处理 |
|---|---|---|
| HAL_OK | 操作被接受/执行成功 | 异步发送还要等待完成 |
| HAL_BUSY | 正在发送、邮箱满、接收队列空或操作暂不可执行 | 根据业务选择稍后重试或丢弃，不做无限忙等 |
| HAL_ERROR | 参数错误、未初始化、配置不匹配、bus-off 或 HAL 错误 | 查状态与统计，修正配置或执行恢复 |
| HAL_TIMEOUT | HAL 阻塞操作超时 | 记录故障并按业务恢复 |

`USART_Read()` 返回读出的字节数，0 表示当前没有可读字节或参数/状态不满足要求。需要区分原因时结合 `USART_GetStats()` 与初始化结果。

## CAN

### 初始化修正

原代码的过滤器结构体未清零，首次配置 CAN1 前没有设置 `SlaveStartFilterBank`。现在使用零初始化，设置分界14后才分别配置 CAN1 bank0、CAN2 bank14。

硬件过滤器接受全部帧，软件仅把标准数据帧送入队列，拒绝扩展帧、远程帧和非法 DLC。后续明确电机/其他设备 ID 后可缩小硬件过滤范围。

本次还同步了 `can.c` 中四个配置赋值：CAN1/2 的 AutoBusOff、AutoRetransmission 均为 ENABLE，与已有 `.ioc` 一致。这是生成配置的同步修正，应用代码仍放 USER CODE 区。以后 Generate Code 后核对这四项。

CAN2 保留对 CAN1 时钟的依赖。CAN1/2 RX0 中断继续由生成的 `stm32f4xx_it.c` 转发给 `HAL_CAN_IRQHandler()`。

### 发送保护

`CAN_Transmit()` 只接受本工程 `&hcan1`、`&hcan2`，同时核对 Instance、时钟、BSP 初始化和 HAL LISTENING 状态。

- ID：0～0x7FF。
- DLC：0～8。
- 长度大于0时 data不能为空；允许零长度数据帧。
- bus-off 返回 HAL_ERROR；无空闲邮箱返回 HAL_BUSY。
- 先复制到零填充8字节数组，再调用 HAL，避免 HAL 固定读取8字节时越过短数组边界。
- 邮箱检查和装填在短临界区中完成，防止两个线程竞争同一个空邮箱。

```cpp
uint8_t payload[3] = {1, 2, 3};
HAL_StatusTypeDef status = CAN_Transmit(&hcan1, 0x123, payload, sizeof(payload));
// HAL_OK 表示帧写入邮箱，不代表对端已经接收或应用协议已确认。
```

CAN 外设会把数据复制到硬件邮箱，所以调用返回后源数组可重用。

### 接收队列

HAL 回调读取 FIFO0，每次最多处理3帧，限制 ISR 的工作量。两个总线分别有16帧软件队列，满时丢弃新帧并增加 dropped，保留尚未消费的数据。

```cpp
CAN_RxFrame frame;
while (CAN_Read(&hcan1, &frame) == HAL_OK) {
    // 按 frame.id 和 frame.length 分发给电机/其他模块。
    // 例如收到电机反馈后再由模块解析大端整数、编码器和转速。
}
```

BSP 不直接调用电机单例，避免底层依赖上层。未来电机线程需要消费两个 CAN 队列，再调用反馈更新逻辑。

CAN_Stats：received 是入队帧数；dropped 包括软件拒绝的帧和软件队列满；errors 是读 FIFO 失败次数。这些计数不能完整代替 CAN 硬件错误诊断，不代表已统计全部总线错误或硬件 FIFO 丢帧。

## UART

### 验证哪些配置

仅支持 `&huart1`、`&huart6`，当前支持8-bit、无校验、TX_RX配置。

| 请求 | Stream | Channel | 方向 |
|---|---|---|---|
| USART1 RX | DMA2 Stream2 | 4 | 外设→内存 |
| USART1 TX | DMA2 Stream7 | 4 | 内存→外设 |
| USART6 RX | DMA2 Stream1 | 5 | 外设→内存 |
| USART6 TX | DMA2 Stream6 | 5 | 内存→外设 |

检查 DMA 句柄是否链接到正确 UART，Normal、Byte、MemInc、PeriphInc 等参数，以及对应 UART/DMA NVIC 是否启用。DMA RX 地址须位于普通 SRAM 的 `0x20000000..0x2001FFFF`，整个缓冲区都必须落在范围内。CCM、FLASH 均不作为 DMA RX 缓冲区。

不自动修改错误配置再继续，而是返回错误，便于发现 `.ioc` 或生成代码的问题。

### 三种发送方式

```cpp
const uint8_t text[] = {'O', 'K', '\r', '\n'};
HAL_StatusTypeDef status = USART_Transmit(&huart1, text, sizeof(text), USART_MODE_DMA);
```

| 模式 | 大小限制 | 生命周期与行为 |
|---|---:|---|
| BLOCK | 1～65535字节 | HAL 阻塞发送，默认100ms超时；源数据在调用结束前有效 |
| DMA | 1～256字节 | 复制到BSP内部缓冲区后启动DMA；返回后调用方可重用源数据 |
| IT | 1～256字节 | 同样先复制，然后使用UART中断发送 |

每个 UART 同时只接受一笔发送，没有隐藏发送队列。正在发送时返回 HAL_BUSY，不能覆盖内部缓存。两个 UART 分别管理自己的状态。

阻塞发送不允许在 ISR、全局关中断或 HAL_MAX_DELAY 无限超时条件下执行。线程中它仍会占用执行时间，优先使用 DMA/IT；超时按数据长度和波特率选择。

DMA 完成不一定表示最后停止位已发完：HAL 在 DMA 搬运完成后使用 UART TC 中断收尾。实际 UART handler 已由 `threadx_port.c` 提供，最终 `HAL_UART_TxCpltCallback()` 释放 BSP 发送占用。不能删除 UART NVIC 只保留 DMA IRQ。

### Normal DMA + Idle 接收

每个串口默认提供256字节 DMA 工作缓冲区和512字节环形数组，其中可存511字节。

1. `HAL_UARTEx_ReceiveToIdle_DMA()` 启动接收。
2. 关闭 DMA HT 中断，避免把半包和后续完整事件重复入队；回调仍显式忽略 HT。
3. 出现 IDLE 或满缓冲区 TC 时，HAL 停止本轮 Normal DMA。
4. BSP 将本轮字节复制到环形队列。
5. 再次启动 Receive-to-Idle，恢复接收。

```cpp
uint8_t bytes[128];
// 在负责接收的线程循环里定期执行，不放进阻塞式ISR。
HAL_StatusTypeDef recovery = USART_Service(&huart1);
uint16_t count = USART_Read(&huart1, bytes, sizeof(bytes));
// 把 count 字节送入上层协议解析器。
```

`USART_Read()` 一次最多取256字节，限制关中断时间；有更多数据可再调用。队列满时丢弃新字节，增加 dropped_bytes。

**这是字节流接口，不是协议包接口。** IDLE不保证完整包；长包可以在TC处分段，多个包也可能出现在同一次回调。上层要处理帧头、长度、校验、拆包和粘包。

Normal DMA 在重启接收时有短暂间隙，不能保证连续高速输入完全无丢失。后续确有吞吐需求时，应改为 Circular DMA+读写索引或双缓冲，并同步调整配置与实现；不能直接只把 `.ioc` 改成Circular，本封装会拒绝这种配置。

### 错误和恢复

UART 错误回调记录错误次数、ErrorCode和接收状态。HAL abort/错误路径完成后，由接收线程周期调用 `USART_Service()` 尝试重新装填。回调内不调用可能阻塞的 HAL abort。

HAL的DMA错误路径可能先把UART发送状态设为READY，而TX DMA流仍需停止。BSP不会立刻释放发送缓冲区：保留占用，Service在线程中完成AbortTransmit后才允许下一笔发送；期间到来的旧完成回调也不能提前释放缓存。该异常路径已加入模拟测试。

应用目前只有演示线程，没有真正的串口消费者；启用接收后若不消费数据，队列最终会满，这是预期的可观测结果。后续接收线程同时负责读队列和定期Service。

若接收已经停止，Service不会自动违背Stop指令恢复。DMA处于ERROR等无法正常启动的异常状态会返回错误，需由上层安排停止和重新初始化/复位，不能靠反复忙等掩盖。

### 可选自定义接收缓冲区

通常不需要调用 `USART_Receive()`，默认内部缓冲区已经启动。需要自定义时，先检查 `USART_StopReceive()` 成功，再调用 `USART_Receive()`：

```cpp
static uint8_t custom_rx[128]; // 需由链接脚本放在普通SRAM
if (USART_StopReceive(&huart1) == HAL_OK) {
    HAL_StatusTypeDef status = USART_Receive(&huart1, custom_rx, sizeof(custom_rx));
}
```

缓冲区必须保持有效直到成功停止；不能用即将退出的局部数组。BSP会反复重用该缓冲区，应用应从USART_Read读取副本，不在DMA写入时修改/解析工作缓冲区。

RX 的初始化、Stop、Receive和Service由同一个线程负责。短临界区保护队列和发送状态，**不代替跨线程的整个“停止→换缓冲区→重启”事务锁**。运行期不要绕过BSP直接调用HAL收发/DeInit。

## PWM

### 范围和初始化

支持TIM1 CH1～4、TIM8 CH1～3。其他句柄、TIM8 CH4和非法通道被拒绝；HAL句柄、定时器时钟和BSP初始化必须有效。

`PWM_Init()`只验证，不启动；初始Pulse=0保持配置原状。应用明确设定脉宽后才启动。

```cpp
if (PWM_SetPulseUs(&htim1, 1500.0f, TIM_CHANNEL_1) == HAL_OK) {
    HAL_StatusTypeDef status = PWM_Start(&htim1, TIM_CHANNEL_1);
}
```

这只是接口示例，不会自动执行。舵机中位、端点和接口接线仍按实际型号核对。

### 周期、占空比和微秒

TIM1/TIM8 在APB2；当前PCLK2=84MHz，APB2分频不为1，所以定时器时钟为168MHz。读取当前时钟和PSC，而不是把CPU频率直接当定时器频率。

```text
计数频率 = TIMCLK/(PSC+1)
计数周期数 = round(目标秒数 × 计数频率)
ARR = 周期数-1
CCR = round(占空比 × 周期数)
CCR = round(脉宽微秒 × 计数频率 / 1000000)
```

当前PSC167，一个计数1µs；ARR19999为20ms；CCR1500为1.5ms。

| API | 参数单位/范围 |
|---|---|
| PWM_SetPeriod | 秒；2～65535个计数 |
| PWM_SetDutyRatio | 0～1，包含两个端点 |
| PWM_SetPulseUs | 微秒；不超过当前周期 |

拒绝负数、NaN、Infinity和超出范围的值，不悄悄截断。

TIM1/TIM8为16-bit：本封装把周期数上限设为65535，保留 `CCR=周期数` 表达100%占空比的能力。如果允许ARR65535，周期数65536，100%对应CCR65536会超出16-bit。

### 改周期的限制

一个定时器所有通道共用PSC/ARR。任意通道仍在运行时 `PWM_SetPeriod()` 返回 HAL_BUSY；需先停止该定时器所有使用通道。

改变周期时会按新旧周期比例缩放各通道CCR，尽量保持占空比；停止状态下通过更新事件装载影子寄存器并清除更新标志。**保持占空比会改变舵机脉宽**，所以改周期后需要重新调用SetPulseUs设定目标脉宽。

启动检查HAL通道状态，重复启动返回HAL_BUSY；停止已经停止的通道返回HAL_OK。停止一个通道时，其他已经运行的通道应继续运行，此行为已在模拟测试中覆盖，真实波形仍需上板核对。

## 并发与ISR

`bsp_critical.hpp` 保存/恢复PRIMASK，而不是无条件开中断；嵌套时保持原有屏蔽状态。它只包住有界的寄存器操作、缓冲区复制和队列状态变化。

阻塞UART发送、CAN启动和UART停止放在中断允许的线程/主程序上下文，不在短临界区中执行。CAN接收最多3帧，UART搬运最多256字节，避免ISR无限循环。

这些措施不保证任意应用负载都满足实时性；上板后需测量ISR时间、总线吞吐和消费线程周期。队列解决数据转交，不自动实现协议和任务调度。

## 本机验证与硬件待办

执行记录见 `docs/logs.md`。

```powershell
cd board
cmake --preset Debug
cmake --build --preset Debug
```

主机模拟测试是可选的Linux x86-64检查，不能在MCU上运行，也不要求Windows用户先安装它：

```bash
bash tests/bsp/run.sh
```

测试替换HAL和寄存器，并将测试BSS映射到STM32普通SRAM数值区间，以执行真实的DMA地址范围检查。使用UBSan；不模拟电气、真实DMA、抢占时序或CAN ACK。

覆盖：初始化前拒绝调用、错误句柄、DMA映射/NVIC、过滤器字段、CAN短缓冲区/ID/DLC/邮箱/队列、UART缓存复制/忙状态/半传输/重装填/环形溢出/错误重试、PWM非法浮点/周期/通道/多通道启停、PRIMASK恢复。

目前按用户要求不执行上板测试。后续需要：双CAN真实收发、UART长流量和错误恢复、PWM示波器测量、线程消费队列、接收间隙评估。

## 应用补丁

压缩包中的 `bsp-completion.patch` 相对于上述main基线生成。先解压，把patch放到包含 `board/`、`bsp/` 的工程根目录，然后在该目录：

```powershell
git status
git apply --check .\bsp-completion.patch
git apply .\bsp-completion.patch
cd board
cmake --preset Debug
cmake --build --preset Debug
```

若check失败，先核对分支和本地未提交改动，不强行覆盖。无需重复应用ThreadX旧补丁，也无需再次调整时钟树。

可选提交句子：

```text
feat(bsp): complete CAN, UART DMA and PWM wrappers with initialization guards
```

本次没有替你向GitHub推送。确认本机编译后，再按已有流程commit/push。
