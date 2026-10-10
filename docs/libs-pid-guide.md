# libs：PID、CRC 与基础数值函数

## 基线与完成范围

基于 `hcyhappy/RM_Framework` 的 main `474cef570bb2502bb4fff265851e86fbdfaa1219`，已经包含前一步 BSP。

- 补全 PID_POSITION 和 PID_DELTA。
- 两种模式都限制累计积分贡献与最终输出。
- 保留构造、Tuning、UpdateResult、Clear 接口；新增 GetIntegralOutput 观察累计积分。
- 保留 CRC8/CRC16 表、正常输入的算法结果、校验字节顺序和原有最短报文约束，补充空指针保护。
- 保留 Numeric::LimitABS，补充异常数值和非法限幅保护。

libs 不包含 HAL 和 ThreadX 调用，可以独立在主机测试。本次不设置电机 PID 参数，也不启动电机闭环或 IMU 加热。

## 1. PID 的误差与三个分量

每次 UpdateResult 表示一个控制采样周期，误差为 `e[k] = ref - fdb`。

| 字段 | 含义 |
|---|---|
| ref / fdb | 目标值 / 测量反馈 |
| err[0] | 本次误差 e[k] |
| err[1] | 上次误差 e[k-1] |
| err[2] | 上上次误差 e[k-2] |
| kp / ki / kd | 当前离散采样周期下的增益 |
| maxOut | 最终控制输出的绝对值上限 |
| maxIOut | 累计积分贡献的绝对值上限 |

P 对当前偏差立即反应；I 积累长期偏差，帮助消除稳态误差；D 对误差变化反应，也容易放大测量噪声。负增益允许用于特殊控制方向，但实际系统必须检查反馈极性。

这里的“位置式”指计算形式，不代表只能控制电机位置；位置式也可以用于速度环、温度环。

## 2. 位置式计算

记 `clip(x,L)` 为把x限制到[-L,+L]：

```text
P[k] = kp × e[k]
I[k] = clip(I[k-1] + ki × e[k], maxIOut)
D[k] = kd × (e[k] - e[k-1])
u[k] = clip(P[k] + I[k] + D[k], maxOut)
```

当前模式下：pResult=P，iResult=累计I，dResult=D，result=最终u。积分限的是已经乘增益后的输出贡献，不是裸误差和，也不是结果的占空比。

例如只有ki=2、maxIOut=3，连续误差1时累计积分为2、3、3；误差反向为-1时变成1、-1、-3。每次使用已限幅的积分状态，不在隐藏变量中继续无限积累。

## 3. 增量式计算

把相邻两次位置式相减，在没有输出截断等差异时可以得到：

```text
ΔP[k] = kp × (e[k] - e[k-1])
ΔD[k] = kd × (e[k] - 2×e[k-1] + e[k-2])
I[k]  = clip(I[k-1] + ki × e[k], maxIOut)
ΔI[k] = I[k] - I[k-1]
u[k]  = clip(u[k-1] + ΔP[k] + ΔI[k] + ΔD[k], maxOut)
```

未触及积分上限时，ΔI就是ki×e。触及上限后，用真实允许的积分变化量，避免继续把已经被积分限幅禁止的贡献加到输出。

**字段语义区别：**增量式的pResult、iResult、dResult是本次增量；result仍是累计后的最终输出。两种模式都通过GetIntegralOutput取得累计I。

不能用 `abs(iResult) <= maxIOut` 检查增量式积分是否合规。例如累计I从+3变到-3，本次ΔI为-6；累计I始终在[-3,+3]内，但增量可以达到两倍上限。

增量式不能只返回Δu给执行器，执行器通常需要累计的result。也不能在外部再把result累加一次。

## 4. 限幅与饱和的边界

- 输出限幅保证result在[-maxOut,+maxOut]。
- 积分限幅保证GetIntegralOutput在[-maxIOut,+maxIOut]。
- maxOut=0表示最终输出锁为0；maxIOut=0表示禁用积分贡献。
- 负数、NaN、Infinity不是合法上限，不会取绝对值或悄悄放开限制。

这是积分限幅方案，不是完整的条件积分或反算抗饱和。若maxIOut远大于执行器能力，积分仍可在这个有限范围内积累，造成恢复延迟。将来应结合电机电流上限、实际执行器饱和和模式切换设计控制保护。

没有输出饱和时，两个模式可按相同误差序列得到一致结果；增量式使用上一轮已经截断的输出，饱和以后不保证与位置式完全等价。例如纯P、kp=10、maxOut=5，误差1后改为0：位置式回到0，增量式可能到-5。这不是随机误差，是截断后的状态差异。

计算使用double中间量，防止大但有限的增益乘误差在限幅之前发生float溢出。最终状态仍为float。极端情况下pResult/dResult等诊断分量会限制到float可表示范围；最终输出用未截断的double分量计算，不能拿这些诊断字段重新求和代替result。

Cortex-M4的FPU主要支持单精度，double计算会引入软件运算开销。代码目前优先保证边界行为；实际高频闭环应测量UpdateResult耗时，而不是仅凭编译成功判断满足实时要求。

## 5. 控制周期必须固定

UpdateResult不读取时间，不自动乘除dt。若从连续PID参数转换，固定采样周期Ts（秒）时通常：

```text
kp = Kp
ki = Ki × Ts
kd = Kd / Ts
```

例如从1ms改成10ms采样，通常不能原封不动沿用ki、kd。当前代码中的ki、kd是离散系数，既有参数也必须按其原本定义理解。

后续控制线程应按明确周期调用，不能让UART/CAN消息频率随意决定PID积分时间。不要用任意HAL_Delay忙等替代RTOS的周期与同步设计。

## 6. 状态清理、调参与错误输入

Clear保持原有行为：清除ref、fdb、误差历史、三个分量、最终输出和累计积分；保留增益、上限和mode。调用后需要重新填写目标和反馈。

有效Tuning更新三个增益但不清状态；输入任一增益非有限时，整组调参被拒绝，原增益保持不变。调参和切换模式不提供无扰切换保证，特别是增量式在误差不变时不会把新kp的作用直接当作一个全新的位置式输出。

检测到mode变更时，会清掉旧模式历史和输出，保留当前ref/fdb，再计算新模式第一步。构造器的非法int模式不会通过uint8窄化错误地变成合法模式。

目标、反馈、增益、上限或必要历史出现NaN/Infinity，以及误差差值超出float表示范围时，输出与内部历史归零。当前目标与反馈保留，修复输入后可继续计算；上层仍应记录传感器故障，不能把0输出当成通信正常的证明。

第一次更新/清零后历史误差为0，非零目标可能产生D项跳变，即微分冲击。当前不包含微分滤波、对反馈求导、角度回绕误差或自动调参。

一个PID对象由一个控制线程维护；若其他线程要调参，通过消息或显式同步转交，不在UpdateResult运行中修改公共字段。

## 7. 使用示例

```cpp
PID controller(2.0f, 0.5f, 1.0f, 100.0f, 50.0f, PID_POSITION);
// 参数仅用于公式演示，不是M2006调参建议。
controller.ref = 2.0f;
controller.fdb = 0.0f;
controller.UpdateResult();
// P=4，I=1，D=2，result=7。
float output = controller.result;
float accumulated_i = controller.GetIntegralOutput();
```

更换构造器最后参数为PID_DELTA可以使用增量式。不要每轮Clear，否则积分和误差历史永远无法建立。

## 8. CRC 保留与验证

正常输入使用原来的查表算法，没有改变CRC表或协议：CRC8的反射逐位多项式0x8C、初值0xFF；CRC16的反射逐位多项式0x8408、初值0xFFFF，均没有额外最终异或。CRC16附加顺序仍是低字节在前、高字节在后。

测试输入ASCII“123456789”：CRC8得到0x0B，CRC16得到0x6F91。测试还通过独立逐位算法对多个长度交叉检查，并验证数据篡改后校验失败。

| API类别 | 长度含义 | 前提 |
|---|---|---|
| Get_CRC* | 参与计算的数据字节数 | 调用方提供种子；有效缓冲区长度足够 |
| Append_CRC* | 数据加校验字节的总长度 | 预留尾部空间；函数写入尾部 |
| Verify_CRC* | 包括校验字节的总长度 | 缓冲区内容完整 |

Append/Verify保留原来的`length <= 2`拒绝规则：CRC16至少1数据字节加2校验字节；CRC8当前也要求总长至少3，即至少2数据字节。没有把这次PID任务扩展成调整CRC报文长度约定。

Get函数遇到空指针返回传入种子，避免解引用；这不是“空指针报文校验通过”，Verify仍拒绝空指针。返回一个CRC值不能替代上层指针、实际缓冲区长度和协议合法性检查。

通信解析时先检查帧头与长度，确认完整帧和缓冲区边界，再做CRC；CRC也不自动解决拆包、粘包或身份认证。

## 9. Numeric::LimitABS

有效输入限制到[-maxValue,+maxValue]。非法上限或NaN输入返回0；正负Infinity输入在有效上限下饱和。上限0输出0。

基础函数没有HAL依赖，可在控制输出/通信解析的数值检查中使用。它不负责角度归一化、单位转换或范围外的协议错误报告。

## 10. 验证和应用补丁

主机测试：`bash tests/libs/run.sh`（Linux，g++）；使用AddressSanitizer和UndefinedBehaviorSanitizer，关闭当前沙箱不支持的进程泄漏扫描。测试不使用动态分配，不声称完成泄漏检查。

覆盖公式手算、正负积分饱和/反向消退、两种模式未饱和一致性、1万次误差序列的边界、零上限、模式切换、Clear/Tuning、NaN/Infinity、大数、CRC逐位参考、报文损坏、空指针和数值函数。

已完成板级Debug与Release构建。固件当前没有调用真正的PID控制链路，链接器可以移除未使用的PID函数；固件大小不增加不代表源码没编译。行为验证来自独立测试，实际控制效果和运行耗时仍待硬件闭环验证。本次按之前约定不执行硬件测试。

把解压出的 `libs-pid.patch` 放在包含board/libs的工程根目录，在VS Code PowerShell终端：

```powershell
git status
git apply --check .\libs-pid.patch
git apply .\libs-pid.patch
cd board
cmake --preset Debug
cmake --build --preset Debug
```

若check失败，先核对分支、基线和本地改动，不强行覆盖。无需重复应用BSP或ThreadX补丁。本次未向GitHub推送。

建议commit：`feat(libs): implement position and incremental PID with output and integral limits`。
