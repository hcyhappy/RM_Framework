# Module 驱动与控制：BMI088、M2006、LED、IST8310

## 1. 本次范围与应用补丁

基线是 GitHub main 提交 `31eec5219a9cf0f3f0e3d957a5d659c6e55ff445`，已包含之前的 BSP 和 PID。本次补全模块源码、增加 I²C/毫秒时钟 BSP 包装、主机测试和说明。LED 原先已经完成，本次核对并测试，不重复替换实现。

在 VS Code 终端进入仓库根目录 `D:\RM_Framework`。先确认工作区干净、之前的修改已提交。解压下载包，把 `module-completion.patch` 放在根目录，再执行：

```powershell
git status --short
git apply --check .\module-completion.patch
git apply .\module-completion.patch
git diff --stat
cd .\board
cmake --preset Debug
cmake --build --preset Debug
```

`git apply --check` 不修改文件；检查成功再应用。遇到检查失败，先核对所在目录和基线，不直接覆盖已有修改。本次不需要重新生成 CubeMX。新 `.cpp` 由已有 CMake GLOB 收集，先 configure 可确保加入构建。

## 2. 实际板级连接

下表依据当前生成的 `main.h`、`spi.c`、`i2c.c`、`gpio.c`，对应标准 RoboMaster C 板。将来换板，应改生成的配置与连接，不能只改软件名称。

| 功能 | 当前连接/配置 | 驱动使用方式 |
|---|---|---|
| BMI088 SPI | SPI1；PB3 SCK、PB4 MISO、PA7 MOSI；Mode 3、8 bit、MSB first、2.625 MHz | HAL_SPI_TransmitReceive，10 ms 有限超时 |
| 加速度计片选 | PA4，CS1_ACCEL，初始高 | 低选中，整段事务结束后恢复高 |
| 陀螺仪片选 | PB0，CS1_GYRO，初始高 | 与加速度计分别控制 |
| BMI088 数据就绪 | PC4 INT1_ACCEL、PC5 INT1_GYRO | 当前普通输入；传感器映射 DRDY，本次无 EXTI/DMA 采样线程 |
| IST8310 I²C | I²C1：PB8 SCL、PB9 SDA，100 kHz | 7-bit 地址 0x0E，BSP 内左移为 HAL 地址 |
| IST8310 复位 | PG6，RSTN_IST8310 | 低有效，初始化拉低 2 ms、释放后等 10 ms |
| IST8310 DRDY | PG3 | 本次读取状态寄存器轮询，不依赖 EXTI |
| RGB LED | PH10 蓝、PH11 绿、PH12 红 | 低电平点亮 |
| 电机 CAN | hcan1 或 hcan2，当前均 1 Mbps | 使用之前的 CAN 接收队列和 CAN_Transmit |

模块不重复调用 MX 初始化；所有 MX_* 和 bsp_Init 应先完成。阻塞 SPI/I²C 操作不能放入 ISR 或关中断区。BMI088、IST8310 由一个传感器线程持有；电机注册、反馈和控制由一个控制线程持有，注册对象生命周期应覆盖线程运行期。

## 3. BMI088：SPI 协议、配置和解析

`ReadReg(sensor, address, buffer, len)` 的 `len` 一律表示有效寄存器数据字节数，返回 HAL 状态。调用者不再自行丢弃 dummy。

- 读地址：`address | 0x80`；写地址：`address & 0x7F`。
- 加速度计读取：发送地址，随后额外发送一个 dummy，再发送 len 个填充字节产生时钟；接收缓冲从偏移 2 复制有效值。
- 陀螺仪读取：发送地址后直接接收寄存器值；有效值从偏移 1 开始。
- 写操作：发送地址和 len 个数据字节，不插入加速度计读协议的 dummy。
- 两种读取都丢弃“发送地址时收到的字节”。加速度计在此之外还多丢弃一个 dummy，不能把两者混淆。

片选跨整个事务保持低，成功、失败和超时均释放。缓冲最多 30 个有效字节；校验参数、SPI 初始化状态、时钟和调用上下文。短临界区只预约总线，不把阻塞 HAL 放进关中断区。预约仅协调 BMI088 实例，其他 SPI1 使用者也应由同一线程统一调度。

`Config()` 先做一次读事务触发加速度计切换到 SPI，再软复位、等待、再次触发 SPI；校验 ACC ID 和 GYRO ID，再写配置并回读。失败时 `Ready()==false`、`INIT_ERR==true`，停止后续初始化。

| 配置 | 写值 | 本实现的选择 |
|---|---|---|
| ACC_PWR_CONF / ACC_PWR_CTRL | 0x00 / 0x04 | Active、加速度计使能 |
| ACC_RANGE / ACC_CONF | 0x01 / 0xAB | ±6g、800 Hz、Normal 带宽 |
| GYRO_RANGE / GYRO_BANDWIDTH | 0x00 / 0x02 | ±2000 °/s、1000 Hz、116 Hz 带宽 |
| GYRO_LPM1 | 0x00 | Normal |
| DRDY 映射 | ACC INT1、GYRO INT3 | 低有效推挽；仅映射输出，未增加 MCU 中断处理 |

特别注意：GYRO_BANDWIDTH bit7 只读且总为 1，因此写 0x02 后回读可为 0x82。校验忽略该位。原代码把名为 GYRO_LPM1_SUS 的 0x80 与带宽相或，并不会因此使陀螺仪休眠；该宏语义不清，但 bit7 在带宽寄存器中没有此功能。

三轴数据均为低字节在前的有符号 16 位补码：

```text
acc[m/s²] = signed_raw × (6 × 9.80665 / 32768)
gyro[rad/s] = signed_raw × (2000 × π / 180 / 32768) − 本板陀螺仪零偏
```

这些量是传感器自身轴系的加速度和角速度，不是 roll/pitch/yaw 姿态角。未做安装坐标变换、重力扣除或姿态融合。两部分独立采样，连续读取不能保证同步，也没有“每次调用必有新样本”的保证。

温度是 11 位补码：`u = (MSB << 3) | (LSB >> 5)`，超过 1023 时减 2048，`T = raw × 0.125 + 23` °C。原始值 0x400 是无效温度，返回 HAL_ERROR 并保留旧输出。它的更新频率与运动数据不同。

`VerifyAccData/VerifyGyroData` 检查读取通信结果，未执行芯片内建激励自检。

`ReadAccData`、`ReadGyroData` 保持原有 void 接口，通过相应 DATA_ERR 和 `LastStatus()` 检查；成功才更新调用者和对象数据。温度通过 `LastStatus()` 判断。通信失败时旧数据仍在，调用者必须检查状态，不能把旧值当新测量。

`Calibrate()` 只在 ThreadX 线程中执行；静止采集 1000 次、每次约 2 ms，失败或角速度过大则保留之前的零偏。不再使用其他板子的预设偏置；默认零偏为零。该过程不是完整的六面加速度校准，也不能证明传感器没有缓慢运动。

`TemperatureControl()` 当前保持停用状态；TIM10/加热硬件没有完成板级配置。本次温度解析已完成，未启用加热闭环。

## 4. M2006：注册、反馈、速度环和位置双环

本实现针对 M2006 P36 + C610，减速比 36:1。不把 C620/M3508 的温度格式套用到 C610。

`registerMotor(&motor, &hcan1, 0x201)` 接收的是反馈 CAN ID，而非数字 1。返回 bool，拒绝未知句柄、ID 越界、同槽重复占用、同一对象注册到其他槽。相同注册可重复调用。电调实际 ID 仍需在硬件上设置一致；注册并不能改变电调 ID。

| 项目 | 格式/本实现单位 |
|---|---|
| 电机 1~4 控制 | 0x200，4 个大端 int16，8 字节 |
| 电机 5~8 控制 | 0x1FF，4 个大端 int16，8 字节 |
| 反馈 ID | 0x201~0x208，8 字节 |
| bytes 0~1 | 转子编码器 0~8191，大端 |
| bytes 2~3 | 转子速度，有符号 rpm |
| bytes 4~5 | 转矩电流反馈，有符号原始数；保留手册原始值，不假定 A/LSB |
| bytes 6~7 | C610 保留字节；temperatureFdb 为 NaN，不能用于温度监控 |
| speedSet / speedFdb | 减速后输出轴 rad/s |
| positionSet / positionFdb | 输出轴相对多圈位置 rad，无 [-π,π] 包角 |
| currentSet | C610 电流命令原始单位；±10000 对应 ±10 A |

反馈速度：`rotor_rpm × 2π / 60 / 36`。位置相邻编码器差值超过 4096 时减 8192，小于 -4096 时加 8192，再累计并按 `2π / 8192 / 36` 换算。前提是两次有效反馈之间实际转子运动小于半圈；数据丢失过多不能靠软件恢复真实圈数。

首帧只建立基准，不从零编码器计算虚假转动。位置模式先在确定机械参考后调用 `ZeroPosition()`；它只是把当前输出轴位置记为零，不能自动寻找机械原点。反馈间隔超时或 BSP CAN 队列丢帧时，位置有效性取消、PID 清零、进入 RELAX；恢复后需重新明确模式，位置控制需重新建立参考。

`PollFeedback()` 每总线最多取 16 帧，校验反馈 ID 和 DLC，再分配给注册电机。它是 CAN 队列消费者，不再从 HAL 中断直接调用模块。一个队列不能再由另一个线程同时消费。若总线还承载其他协议，设置 `unhandledFrame` 回调分发；未设置则未匹配报文被忽略。队列容量仍是之前的 16 帧，控制线程须及时轮询，可通过 CAN_GetStats 观察溢出。

`setOutput()`：

1. RELAX、离线、无注册、无有效配置或输入异常 → 零电流。
2. SPD_MODE：速度 PID 的 ref=speedSet、fdb=输出轴 speedFdb，结果为电流命令。
3. POS_MODE：位置 PID 计算目标速度，再送入速度 PID；位置环 maxOut 的单位是 rad/s，速度环 maxOut 的单位是电流命令。
4. 模式变化清理两个 PID 历史，最终命令限制在 `min(maxCurrent, 10000)` 内。

PID 增益默认仍为零、maxOut 默认零。代码不替你猜测实体负载的增益。根据实际任务设置 speedPid、positionPid 和 maxCurrent，先整定速度环，再整定位置环。上次 PID 的 ki/kd 是离散系数，改变控制周期必须重新换算或整定。通常控制线程周期可先固定 1 ms，但本次未测 MCU 上的真实耗时和调度抖动。

`sendControlData()` 重打包各组：未注册槽为零，只发送存在电机的组，发送前再次检查在线/模式并限幅。返回 HAL_OK 表示所有所需组进入邮箱；HAL_BUSY/ERROR 是有组未成功，不表示原子地发送四组。下一周期应重新计算最新命令，不堆积旧电流报文。离线判断基于 TIM6 毫秒差值，默认 100 ms；设定范围 1~1000 ms。它在代码被调用时生效，并非独立硬件看门狗。

建议后续 threads 阶段的顺序为：`PollFeedback → 模式/目标更新 → 各 motor.setOutput → sendControlData → 固定周期等待`。本次只完成 module，没有启动该线程，也没有发出电机命令。

## 5. LED 与 IST8310 使用

LED：`LED::Set(LED::Color::Red, true)` 点亮红灯，false 熄灭；`LED::Toggle(LED::Color::Green)` 翻转绿灯。GPIOH 未开时钟时不访问端口，非法颜色不操作引脚。LED 已在 heartbeat 中使用。

IST8310 新增 `IST8310.hpp/.cpp`：

```cpp
IST8310 compass;       // 放到持久对象中，由传感器线程唯一持有
MagneticField field;
// MX_I2C1_Init / GPIO 初始化之后：
const auto init_status = compass.Init();
// 后续周期调用，仅 HAL_OK 才使用 field；HAL_BUSY 表示仍在等待单次测量。
const auto read_status = compass.Read(&field);
```

Init 硬复位、校验 WHO_AM_I=0x10、写 CNTL1=0x01 触发单次测量。Read 等待至少 5 ms，单独读 STAT1；DRDY 未置位返回 HAL_BUSY，不提前读取 XYZ 清除就绪标志。准备好后读取 0x03~0x09，再触发下一次转换；有 DOR 或通信失败则返回错误、保留旧输出。

XYZ 为小端有符号 16 位，按手册分辨率 0.3 µT/LSB 换算，输出传感器本体轴系磁场。手册还给出约 3.3 LSB/µT，两个是经过取整的标称数，不应当看成精确互逆。本实现采用 0.3。STAT2 bit3 是 INT，不是某些磁力计的 HOFL，不能因此丢弃有效样本。未做硬铁/软铁标定和航向角融合。

I²C HAL 调用置于 `bsp_i2c.cpp`，检查 I²C1/3 句柄、7-bit 配置、时钟、状态、长度、上下文；读写有限超时。这里的地址参数保持未左移的 7-bit 值，避免重复左移。

## 6. 验证与剩余工作

已完成：Arm GNU 13.2.1 的板级 Debug/Release 编译、强制保留模块入口的额外链接检查；HAL 模拟 + UBSan 验证实际模块源码，并回归 BSP 与 libs 测试。默认演示未引用的模块仍可能被链接器移除，所以另做链接检查，不能只凭默认固件大小宣称模块已运行。

主机测试覆盖 dummy、ID/配置回读（含带宽 bit7）、三轴负值和量程、温度无效值、SPI 超时后释放片选、IST 状态/单次转换/错误、注册冲突、编码器双向回绕、速度与位置双环、电流打包限幅、离线与 CAN 队列丢帧保护、报文分发和 LED 极性。

Linux/WSL 安装 g++ 后可运行：

```bash
bash tests/modules/run.sh
bash tests/libs/run.sh
bash tests/bsp/run.sh
cmake --preset Debug -B build/ModuleLinkCheck \
  -DCMAKE_PROJECT_board_INCLUDE="$PWD/tests/modules/force_link.cmake"
cmake --build build/ModuleLinkCheck
```

按约定跳过上板验证。真实 SPI/I²C 电气时序、采样新鲜度、安装方向、CAN 负载、负载下 PID、机械原点、线程周期/栈余量仍待硬件确认。线程接入和整定属于下一阶段。

## 7. 资料

- [Bosch BMI088 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)，配置/温度/陀螺仪带宽/SPI 章节。
- [DJI C610 用户手册](https://rm-static.djicdn.com/tem/17348/RoboMaster%20C610%20Brushless%20DC%20Motor%20Speed%20Controller%20User%20Guide.pdf)，CAN 消息章节。
- [DJI M2006 产品参数](https://www.robomaster.com/en-US/products/components/detail/1277)，减速比。
- [iSentek IST8310 原厂数据手册 v1.0 镜像](https://wiki.hshl.de/wiki/images/1/1c/IST8310_Datasheet_v1.0.pdf)，单次测量、分辨率及 STAT1/STAT2。原厂下载链接当前不可用，本次用原厂手册的镜像核对。
