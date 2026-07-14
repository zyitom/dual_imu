# DM-FC01 双 IMU 最终方案与当前状态

本文只讨论 BMI088、ICM45686、STM32H743 组成的双 6 轴姿态系统。运行时输入仅来自
这两颗 MEMS，不依赖其他信号。板级连线以 `DM-FC01_pinout.xlsx` 为唯一最终依据，
算法细节见 [IMU_FUSION.md](IMU_FUSION.md)，当前恢复点和待办见
[PROJECT_STATUS.md](PROJECT_STATUS.md)。

文中严格区分：

- `定稿目标`：设计已经确定，但代码或板测可能尚未完成。
- `已实现`：代码存在且对应主机测试通过。
- `已板测`：已在当前硬件上得到数据证据。
- `阻塞`：完成前不能宣称达到最终输出规格。

## 1. 最终结论

1. 四路原始流分别采集，两个 IMU 各自运行一个 6 状态 MEKF lane，selector 只在
   两个 gyro lane 之间选择，不对两颗陀螺做固定平均。
2. 时间链采用 `sensor time/硬件捕获 -> TIM5 -> 公共事件时间窗`。这个方向正确，
   四路 FIFO/DMA 已接入 manager；绝对 filter/group delay 尚无已知运动基准可标定。
3. MEKF、重力更新和 selector 定为 `400 Hz`；高频姿态预测与发布定为 `1600 Hz`；
   内部积分消费选中 lane 的全部 `2000 Hz` 或 `3200 Hz` gyro 样本。
4. 四元数是主姿态输出。欧拉角只是 ZYX 派生量，接近 pitch `+/-90 deg` 时必须标记
   奇异。roll/pitch 只在加计主要反映重力时可观，yaw 只有相对意义并长期漂移。
5. 本系统只有两个 gyro lane。两者软不一致而内部证据不足时，正确状态是
   `AMBIGUOUS + integrity_degraded`，不能假装已经识别坏方。
6. 当前可验证静态姿态、相对时序和数据完整性，并记录相对 yaw 漂移；尚未标定绝对
   动态相位，也不提供绝对 yaw。

当前代码已完成 BMI/ICM FIFO、SPI1/SPI4 DMA、manager 接入和 TIM5_CH3 625 us compare
调度，完整 ARM Debug/Release/RelWithDebInfo 固件可链接，十二组宿主测试通过。当前实板 1 s 诊断窗
的代表计数约为 `1608-1612/1997-2000/1610-1612/3219-3223 Hz`；名义配置依次为
`1600/2000/1600/3200 Hz`，差异包含传感器时钟容差和 1 s 计数窗量化。MEKF 为
`399-401 Hz`，fast quaternion/Euler 为 `1598-1602 Hz` 且有效输出率相同；compare
miss/drop 为 0，正常运行发布延迟峰值小于 `20 us`。BMI accel 启动 phase 建立产生
一次预期 timeline reset；稳态 clock stale/causal reset 和 ICM discontinuity 均为 0。
静置板测已进入内部 ZARU，`stationary_confirmed=1`、两 lane 易失 bias trim 均收敛；
噪声尖峰时判据会保守退出并重新驻留，已收敛 trim 不因此清除。

compare ISR 以精确 scheduled epoch 直接生成并发布当期姿态；若 ISR 已跨过后续 compare，
调度器保持原始相位、累计 missed/drop，超过 250 us 的 frame 必须 invalid。
`state.quaternion/euler_rad` 只在
有效 fast frame 时更新，400 Hz MEKF 结果另存于
`state.mekf_quaternion/mekf_euler_rad`。上述结果证明当前板上的目标频率、FIFO 数据链、
时钟映射和 1600 Hz 发布调度已经闭合；它不替代逐台标定、绝对 pipeline delay 标定或
2 h 连续长跑。

## 2. 最终 Hz 配置

以下是本项目唯一默认档，不再保留“为了迁就阻塞读取而降频”的配置：

| 通道/输出 | 名义配置频率 | 片上低通 | 量程 | 数据路径 | 选择理由 |
|---|---:|---:|---:|---|---|
| BMI088 accel | 1600 Hz | OSR4，名义约 145 Hz | +/-24 g | FIFO + 24-bit sensor-time | BMI 最高 ODR；WTM=28 bytes，名义 4 个 7-byte 帧 |
| BMI088 gyro | 2000 Hz | 230 Hz | +/-2000 dps | FIFO + PA1 TIM5 capture | BMI 最高 ODR；逐样本硬件时标 |
| ICM45686 accel | 1600 Hz | ODR/16，名义 100 Hz | hi-res 固定 +/-32 g | 20-byte hi-res FIFO | 每个 2.5 ms 窗约 4 样本 |
| ICM45686 gyro | 3200 Hz | ODR/16，名义 200 Hz | hi-res 固定 +/-4000 dps | 20-byte hi-res FIFO | 每窗约 8 样本，WM=2 |
| MEKF + selector | 400 Hz | - | - | 2500 us 公共窗 | 重力更新和协方差校正 |
| 快速 quaternion/Euler | 1600 Hz | - | - | 625 us 固定 TIM5 网格 | 每个 MEKF 窗正好 4 次输出 |
| 内部 gyro 积分样本 | 2000/3200 Hz | 额外软件 LPF 默认关闭 | 同选中 lane | 全部样本保留 timestamp | control API 是 latest snapshot，不是逐帧发布接口 |

姿态输出选 `1600 Hz`，不是 `1000 Hz` 或 `2000 Hz`：它满足“大于 1000 Hz”，625 us
可由 1 MHz TIM5 精确表示，并与 400 Hz 窗形成严格的 4:1 epoch 网格；同时长期平均
对应每个 ICM accel 样本和约 2 个 ICM gyro 样本。改成 2000 Hz 不增加重力观测信息，
ICM 被选中时反而会增加没有新批次可用的短时外推次数。

### 2.1 为什么不是所有 ICM 通道都设为 6400 Hz

BMI accel `1600 Hz` 和 gyro `2000 Hz` 已经是器件上限。BMI 还能改变的是带宽：
gyro `230/532 Hz`、accel `OSR4/OSR2/Normal`，不是更高 ODR。

ICM accel/gyro 的低噪声模式支持 6400 Hz，但本项目选择 1600/3200 Hz：

- 100/200 Hz 名义低通下，当前 ODR 已分别提供 16 倍采样比。
- 3200 Hz gyro 在一个 625 us 快速输出周期内长期平均约 2 个样本，在 2.5 ms MEKF
  窗内长期平均约 8 个样本，足够做分段积分和二阶圆锥补偿。ICM 时钟与 TIM5 未
  锁频，单个窗口的实际帧数允许随相位变化。
- 6400 Hz 若仍要约 200 Hz gyro 低通，必须从 `ODR/16` 改成 `ODR/32`；直接使用
  `6400 + ODR/16` 得到的是约 400 Hz，而不是 200 Hz。
- 6400 Hz 会把 20-byte FIFO 流量约从 64 kB/s 增加到 128 kB/s。它可能降低离散
  积分步长，但在没有动态参考数据时没有证据证明这项收益大于额外复杂度。

所以 1600/3200 Hz 是固定默认值，不宣称是对所有载体都普遍最优。若未来要求更高
动态带宽，必须重新给出相位、噪声和总线预算，不能只因为“数字更大”升档。

### 2.2 ICM hi-res 换算

ICM 20-byte high-resolution FIFO 中，gyro 为 20-bit，accel 有 19 个有效位且最低位
固定为 0。hi-res FIFO 字段的比例不受 UI FSR 选择影响，固定为：

- gyro: +/-4000 dps，约 `131.1 count/(dps)`，约 `0.00763 dps/count`。
- accel: +/-32 g，`16384 count/g`，有效最小步进约 `122.1 ug`。

换算必须使用 hi-res 专用比例，不能沿用 16-bit UI 寄存器比例。

### 2.3 带宽结论的边界

`145/230/100/200 Hz` 是数据手册或寄存器给出的名义设置，不是当前板上实测完整频响。
“所有刚体信号都低于 100-150 Hz”不是普适定律；同样，数字低通能减少高频输出，
但不能据此保证消除所有 vibration rectification error。当前默认档是在没有目标载体
频响数据时的保守选择，不承诺绝对动态相位指标。

## 3. 最终引脚与 MCU 配置

| 网络 | MCU | 最终配置 | 说明 |
|---|---:|---|---|
| `BMI088_0_ACS` | PA2 | GPIO push-pull，空闲高，no-pull，low speed | BMI accel SPI1 CS |
| `BMI088_0_GCS` | PA3 | GPIO push-pull，空闲高，no-pull，low speed | BMI gyro SPI1 CS |
| `BMI088_0_ADR` | PA0 | TIM5_CH1 AF2，上升沿，no-pull，filter=0 | accel FIFO WTM=28 bytes，及时读空时名义约 400 IRQ/s |
| `BMI088_0_GDR` | PA1 | TIM5_CH2 AF2，上升沿，no-pull，filter=0 | gyro DRDY，名义约 2000 capture/s |
| `ICM45686_1_ACS` | PC13 | GPIO push-pull，空闲高，no-pull，low speed | ICM AP_CS，访问 accel/gyro/FIFO |
| `ICM45686_1_GCS` | PC2_C | GPIO push-pull 恒高，no-pull，low speed | AUX1_CS 始终去选通；闭合 PC2 analog switch |
| `ICM45686_1_ADR` | PE4 | EXTI4 上升沿，no-pull | ICM INT1/FIFO WM=2，名义约 1600 IRQ/s |
| `ICM45686_1_GDR` | PE3 | 输入，no-pull，默认不启用 IRQ | ICM pin9/INT2，最终基线不使用 |
| TIM5 | internal | 1 MHz，32-bit + 软件扩展 64-bit | 全系统微秒 epoch |

SPI1、SPI4 使用 mode 3、约 6 MHz。SPI DMA 已在 `.ioc` 中分配：SPI1 RX/TX 使用
DMA1 Stream3/4，SPI4 RX/TX 使用 DMA1 Stream5/6。TIM5 IRQ 优先级 1，PE4 EXTI
优先级 2，SPI RX/TX DMA 优先级 3/4；TIM5 capture 必须高于数据搬运完成中断。
M7 I-cache 已启用，D-cache 明确关闭；当前 DMA buffer 没有配套 cache clean/invalidate
协议，在完成一致性设计前不得启用 D-cache。

### 3.1 ICM 的 ACS/GCS 含义

ICM45686 是单颗 6 轴器件。PC13 的 `ACS` 对应 AP_CS，一根主片选即可读取 accel、
gyro 和 FIFO。PC2_C 的 `GCS` 实际对应 AUX1_CS，不是“gyro CS”；AUX1 不用于本
项目且不能访问主 FIFO，因此由 GPIO 恒高、始终去选通。这是最终禁用状态，不是引脚
方案尚未决定。固件同时用 override 显式禁用
AUX1、关闭 virtual AUX1 access，并在上电时回读确认，不只依赖 CS 为高。

### 3.2 BMI accel 只启用 INT1

最终只把 accel FIFO watermark 映射到 INT1/PA0，INT2 不启用。把同一中断同时映射
到 INT1 和 INT2 只是复制同一信号，不增加样本或时间信息。Bosch DataSync 需要板上
把一个 gyro DRDY 输出物理接到一个 accel 同步输入，并用另一路通知 MCU；当前 XLSX
只有各传感器到 MCU 的网络，没有传感器间交叉网络，因此本板不启用 DataSync。

BMI gyro 已把 PA1 配成逐样本 DRDY 时间源，同时使用 stream FIFO 缓冲。FIFO 中保留
`WM=1` 水位值但关闭 watermark interrupt。TIM5_CH2 ISR 将每个 capture 的
`(timestamp, sequence)` 直接写入 ring。启动时先保持 gyro FIFO BYPASS，等待 gyro 进入
稳定 NORMAL 后捕获并消费一个真实 DRDY 作为边界，再在下一次采样前切到 STREAM；若切换
事务期间已有新边沿则清 FIFO 后重试。这样 STREAM 中第一个 frame 和 ring 中第一个待消费
capture 从同一物理采样边界开始。

运行时先读 FIFO frame count N，再读取 oldest-first 的 N 个 frame，并从 ring 原子取最老
N 个 capture；读取期间到来的新边沿留在队尾。capture 不足、sequence/timestamp 不连续、
FIFO truncate/overrun、已启动的 FIFO_DATA DMA 失败或队列 overflow 都会推进已知物理
sequence、丢弃本批并锁存 `GYRO_TIMING`，停止该 lane 的后续 gyro DMA。当前基线不做不安全
的自然恢复，复位后重新执行上述边界同步。

### 3.3 未使用的 ICM pin9

最终系统没有同步或触发输入。ICM pin9 保持 INT2 功能，PE3 保持 MCU 输入但不映射
中断；FSYNC 和 CLKIN 均关闭。两颗 MEMS 自由运行，统一通过各自 sensor-time/capture
映射到 TIM5，不存在用脉冲触发一次转换的路径。

### 3.4 CubeMX 重新生成边界

引脚、TIM5、SPI、DMA 和 NVIC 由 `FirstFlightControler.ioc` 管理。SPI DMA 使用的
`0x30040000..0x300407ff` 固定保留区不属于 `.ioc` 能完整表达的内容，因此由项目自有
`linker/dual_imu_h743.ld` 管理；GCC 与 Clang toolchain 均固定引用该文件，CMake 在配置
阶段检查它确实生效，linker 再断言保留区起止地址。CubeMX 即使重新生成默认
`STM32H743XX_FLASH.ld`，构建也不会使用该默认文件，不能静默覆盖 IMU DMA staging。

当前 `.ioc` 已经包含最终 USB CDC、PA11/PA12、PLL3 48 MHz、TIM5、SPI DMA、NVIC、
I-cache on 和 D-cache off 配置。HI91 帧、CRC 和发送队列是应用层逻辑，不需要在
CubeMX GUI 中新增配置；USB FS bulk endpoint maximum packet size 保持 64 bytes，
不能改成 82。当前 1600 Hz、82-byte 满负载板测下，OTG_FS IRQ 优先级 0 时 TIM5
compare miss/drop 仍为 0、发布延迟峰值 19 us，因此暂不调整优先级。

每次 Generate Code 后必须确认：TIM5 仍为 1 MHz/32-bit 且 CH1/CH2 capture、CH3
compare 保留；SPI1 DMA 为 Stream3/4、SPI4 DMA 为 Stream5/6；`app_init()` 位于
`MX_TIM5_Init()` 之后；I-cache 只启用一次且 D-cache 关闭；两个 toolchain 仍引用
`linker/dual_imu_h743.ld`；`.imu_dma` 仍为 `0x30040000..0x300407ff`；CDC 自定义代码
仍在 USER CODE 区。随后重新运行十二组主机测试、三个 ARM build 和短时 USB 板测。

2026-07-15 已实际执行一次 CubeMX Generate Code 并完成软件回归：TIM5 CH1/CH2/CH3、
SPI1/4 DMA Stream3/4/5/6、IRQ、CS/EXTI、USB CDC、初始化顺序及所有 CDC USER CODE
均保留。Generate 重新创建了默认 `STM32H743XX_FLASH.ld`，但三个构建仍只使用项目
linker；map 中 `.imu_dma` 仍精确为 `0x30040000..0x300407ff`。再生成后发现并清理了
USER CODE 中重复的 I-cache enable、USB voltage detector 和 SPI KeepIOState 赋值；
配置现在分别只由 `.ioc`/CubeMX 生成，SPI 初始化后只保留寄存器断言。十二组主机测试
和 Debug/Release/RelWithDebInfo 均重新通过。目标板 USB 数据口本次未枚举，因此这版
再生成固件的上板 USB 冒烟仍待补做，不能与此前的实板结果混写为已完成。

## 4. 最终时间戳链

全局时间定义为 TIM5 64-bit 微秒时间：

```text
sensor counter/raw capture
        -> unwrap
        -> affine map to TIM5
        -> optional measured relative delay correction
        -> per-sample event timestamp
        -> common 2500 us windows
        -> 625 us fast-output grid
```

### 4.1 BMI088 accel

BMI accel 24-bit sensor-time 分辨率为 `39.0625 us`，1600 Hz 下名义每个样本相隔
`16 ticks`。sensor-time 不是每个 FIFO 数据帧自带的时间戳；驱动解析 FIFO 控制帧，
但只有恰好 4 个有效 accel frame、28 个数据字节、无 skip/drop/truncate 的水位批次
才允许建立时钟锚点。

首个合格批次从 sensor-time 建立 16-tick 采样 phase，并用 PA0/TIM5_CH1 硬捕获的
watermark 时刻观察 sensor clock 到 TIM5 的 affine 映射。此后正式 watermark sample
tick 直接定义为 `last_sample_tick + 4 * 16`，不再让 DMA 完成时读取的 sensor-time
快照覆盖连续样本序列。批内四帧再按 16 ticks oldest-first 回填 epoch。
PA0、FIFO request 和 DMA complete 都不直接冒充四个样本的测量时刻。结构非法的批次
不发布，并清除采样 phase/时钟映射；当前板启动建立相位时 `timeline reset=1`，稳态不再
增长，clock reset 和 causal failure 均为 0。

### 4.2 BMI088 gyro

PA1/TIM5_CH2 捕获每个 gyro DRDY。FIFO 提供抗阻塞缓冲，capture queue 提供时间。
稳定 NORMAL 阶段以 BYPASS 中的一个真实 DRDY 建立边界，再在下一边沿前切 STREAM；
运行时 FIFO 与 capture ring 都按 oldest-first 消费 N 项。任一数量、序列、时间或 DMA
完整性不匹配都会锁存时序故障，不能等待未来边沿，也不能凭固定 ODR 补造“精确”时间。

### 4.3 ICM45686

普通 20-byte FIFO 帧携带 16-bit、1 us timestamp，正常每 `65.536 ms` 回卷。最终
必须显式配置并回读
`TMST_DELTA_EN=0`，使字段保持滚动计数器语义；保留 `FIFO_TMST_FSYNC_EN=1` 以在
未启用 FSYNC 时插入普通 timestamp。先解卷，再用 PE4 watermark 批次锚点拟合
`t_mcu = alpha * t_sensor + beta`。ICM 使用 watermark equal 模式；PE4 边沿的所有权
固定属于 oldest-first FIFO 中索引 `WM-1` 的包，即 WM=2 时的第 2 帧。DMA 开始前又有
新包入 FIFO 不改变这个所有权，因此干净批次只要求 `count >= 2`，不能把批次最后一帧
当作锚点。至少接受 4 个锚点后才允许映射有效；timestamp 倒退、header 非法或 overflow
时拒绝锚点并进入恢复流程。当前实板 reset/discontinuity/causal error 均为 0。

### 4.4 时间标定边界

手动往返转动可以通过双 gyro 互相关估计相对时差；小时间偏差产生的瞬时残差近似为
`angular_acceleration * delta_t`，而随 `|omega|` 增长的残差更接近 scale 或安装误差。
这种检查不能测出四条通道相对真实物理运动的共同 group delay。四个 pipeline delay
宏目前均为 0；当前只校正可识别的相对时差，因此绝对 timing 仍未标定，不承诺绝对
动态相位。

## 5. 姿态解算与 1600 Hz 输出

每颗 IMU 独立形成 lane：状态为姿态误差 3 维和本 lane gyro bias 3 维。每个
2500 us 公共窗包含所有原始样本，分段按真实时间积分并加入二阶圆锥项；这是一种
受采样和插值误差限制的近似，不称为“数学上精确积分”。

MEKF 在窗中点使用门控后的重力方向更新，在窗尾输出校正四元数。快速输出层以最新
窗尾四元数为锚，把窗尾之后所有选中 lane 的去偏 gyro 增量继续积分到 625 us 输出
epoch；若该 epoch 之后的样本尚未通过 DMA 到达，则用最新去偏角速度作短时零阶保持
外推：

重力观测只有 rank 2。加计更新把姿态误差和 gyro-bias 两个 Kalman gain block 都投影
到 body-frame 重力切平面，修正量和 Joseph 协方差更新使用同一投影后 gain。这样
covariance cross-correlation 不能把加计噪声注入 yaw 或重力轴 bias；这是保留不可观
gauge，不是让 yaw 获得了观测。

```text
q_fast(t_out) = q_mekf(t_anchor) * Product(Exp(delta_theta_i - b_g * delta_t_i))
```

新 MEKF 锚点到达时，快速层必须从该 epoch 重新播放后续 gyro 增量，避免重复积分。
其中 latest gyro 是已经收到且不晚于输出 epoch 的最新有效样本。
`prediction_horizon_us = output_timestamp_us - latest_gyro_timestamp_us`，定稿最大值为
`3000 us`；选中 lane 从 anchor 到输出 epoch 的时钟或样本覆盖无效时不再外推，输出
invalid，未选 lane 的故障不能停止健康选中 lane。每个 625 us tick 必须独立发布，
`DUAL_IMU_ATTITUDE_PUBLISH_DEADLINE_US=250`；超过 `output_timestamp_us + 250 us` 的帧置
`deadline_miss + invalid`，不能成批晚发或重复上一姿态来伪装 1600 Hz。

lane 切换瞬间用完整 alignment 保持 quaternion 连续，再把 alignment 分为 world-Z yaw
twist 和可观 tilt：只按 `1 rad/s` 衰减 tilt，yaw twist 永久保留。没有航向观测时，
不能强迫新 lane 的相对 yaw 追上旧 lane。`alignment_blend_active=false` 只表示 tilt
已经收敛，不表示 yaw alignment 为 0。若新 lane 缺少 anchor 到输出 epoch 的共同时间
覆盖才输出 invalid；接口另带实际 `publish_timestamp_us` 和 `publish_latency_us`。
当前 `dual_imu_get_attitude()` 的每帧输出携带：

- `quaternion[4]`，body-to-NED 主输出。
- `euler_rad[3]`，顺序为 roll/pitch/yaw 的 ZYX 派生输出。
- `gyro_rate_rad_s[3]`，与该 frame 预测积分一致的选中 lane 去偏角速度。
- `anchor_timestamp_us`、`latest_gyro_timestamp_us`、`output_timestamp_us`。
- `prediction_horizon_us`、`publish_timestamp_us`、`publish_latency_us`。
- `sequence`、`selected_source`、`integrity_flags`、`predicted`、`degraded`、
  `deadline_miss`、`euler_singular`、`valid`。

这里的 `valid` 只证明数值、时间链和样本覆盖满足接口约束，不表示逐台标定已加载、
ZARU 已收敛或姿态精度已经达标。

公开常量 `DUAL_IMU_ATTITUDE_OUTPUT_RATE_HZ`、
`DUAL_IMU_ATTITUDE_OUTPUT_PERIOD_US` 和
`DUAL_IMU_ATTITUDE_PUBLISH_DEADLINE_US` 分别固定为 `1600/625/250`，后续硬件调度必须
直接复用，不得再维护第二套频率常量。

固定 1600 Hz 输出不会增加 400 Hz 重力校正的信息率；它的价值是降低姿态发布等待
并保留高频 gyro 动态。当前 predictor 已实现按真实时间戳的梯形积分、二阶圆锥项、
重锚重放、短时零阶保持、lane 切换半球连续、3000 us horizon 和 85 deg Euler 奇异
标志；它不会把 400 Hz 的重力观测伪装成 1600 Hz 新观测。

### 5.1 “正确欧拉角”的准确含义

- 静止或低动态时，经过标定的 roll/pitch 可由重力约束收敛。
- 持续线加速度与倾斜只靠两颗 6 轴 MEMS 无法完全区分，动态 roll/pitch 不能保证
  绝对正确。
- yaw 初始值定义为 0，仅输出相对变化；绝对 yaw 和沿重力轴 gyro bias 长期不可观。
- ZYX 欧拉角在 pitch 接近 `+/-90 deg` 时奇异，四元数仍可用；`|pitch| >= 85 deg`
  时应置 `euler_singular`，控制算法不得依赖此时的 Euler yaw/roll。

当前启动在首个两 lane 都完整的 2.5 ms 公共窗内，用有效 accel 均值初始化 roll/pitch，
yaw 定义为 0，gyro bias 以 0 起步。当前板没有加载逐台标定，因此启动姿态、零偏和
静置漂移不能作为最终精度指标。之后由两颗 MEMS 自身的低速率、窗内及跨窗方差、
加速度模长/双 lane 一致性和重力方向稳定性连续驻留 2 s 后启用三轴 ZARU，不需要其他
输入。统计先预热 40 窗；驻留 gyro 均值还必须逐轴不超过 `0.5 dps`。实测 temporal
gyro 总 RMS 门按 lane 定为 BMI `0.8 dps`、ICM `0.25 dps`，accel 总 RMS 门为
`0.10 m/s^2`。ZARU 对当前 calibration pipeline 输出的 target 逐轴限制为
`+/-0.5 dps`，bias correction 三维向量的 slew 上限为 `0.1 dps/s`。`lane_calibrated`
只表示本次上电的易失 bias trim 收敛，不表示 scale、安装或温度标定完成；后续逐台
标定仍应覆盖 accel、gyro 零偏及温度项，在此之前不承诺 yaw 漂移指标。

### 5.2 USB HI91 兼容帧

USB CDC 默认输出固定 82-byte、小端二进制帧，布局与用户给出的 HI91 `0x91` 数据包
一致。它只复用帧布局，不声称 `main_status` 与厂商产品定义相同。

| 帧偏移 | 长度 | 字段 | 当前语义 |
|---:|---:|---|---|
| 0 | 2 | header | `5A A5` |
| 2 | 2 | payload length | `4C 00`，即 76 bytes |
| 4 | 2 | CRC16 | little-endian |
| 6 | 1 | tag | `0x91` |
| 7 | 2 | `main_status` | 本项目状态字 |
| 9 | 1 | temperature | 编码时选中 lane 最新值，deg C，四舍五入并限幅到 int8 |
| 10 | 4 | air pressure | 无传感器，固定 `+0.0f` Pa |
| 14 | 4 | system time | TIM5 输出 epoch/1000 的低 32 bit，ms |
| 18 | 12 | acceleration XYZ | 最近的因果、有效 400 Hz fused specific force，g |
| 30 | 12 | angular rate XYZ | 当期 fast predictor 使用的去偏角速度，deg/s |
| 42 | 12 | magnetic field XYZ | 无传感器，固定 `+0.0f` uT |
| 54 | 12 | roll/pitch/yaw | ZYX Euler，deg |
| 66 | 16 | quaternion W/X/Y/Z | body-to-NED |

CRC 为 CRC-16/CCITT，初值 0、多项式 `0x1021`，依次覆盖 frame bytes `0..3` 和
`6..81`，跳过 bytes `4..5` 的 CRC 字段。给定样例结果为 `0xBB14`，线上按小端发送
`14 BB`。

`main_status` bit 定义如下：

| bit | 含义 | bit | 含义 |
|---:|---|---:|---|
| 0 | attitude valid | 8 | BMI 本次上电 bias 已收敛 |
| 1 | accel valid | 9 | ICM 本次上电 bias 已收敛 |
| 2 | gyro valid | 10 | 当前选择 ICM gyro lane |
| 3 | integrity degraded | 11 | 内部静止已确认 |
| 4 | publish deadline miss | 12 | BMI fault 非零 |
| 5 | Euler singular | 13 | ICM fault 非零 |
| 6 | 两颗 IMU 均初始化 | 14 | 本 DTR 会话发生输出丢帧 |
| 7 | selector healthy | 15 | yaw 为相对航向 |

主机必须置 DTR 才开始新会话；关闭端口时固件清空应用发送队列并持续丢弃未发布的旧
姿态，固件不会主动回放断开期间的数据。主机驱动仍可能保留关闭前已经完成的一小段
传输，因此应用应在同一个打开的文件描述符上设置 raw/DTR、执行 input flush，再从
`5A A5` 重同步并开始验收，不能先用另一个程序短暂打开串口。原始流量为
`82 * 1600 = 131200 bytes/s`。固定
HI91 布局只有 millisecond `uint32_t` 时间，没有 sequence：1600 Hz 下相邻帧会共享
ms 时间，约 49.7 天回卷，也不能仅靠该字段精确定位单帧丢失；内部 64-bit/us API 才是
权威时间语义。accel 的信息率仍是 400 Hz，正常会在四个 fast frame 中重复，并在
时间不因果或年龄超过三个融合窗（7.5 ms）时清零且撤销 accel-valid。
温度以及 initialized/calibrated/stationary/fault 等慢状态位取编码时的最新状态；
quaternion、Euler、gyro、accel-valid 和 deadline 等数据有效性仍按该 frame epoch 判定。

## 6. 观测与仲裁边界

- 加计更新使用 norm、双加计一致性、角速度、角加速度、样本完整性和 MEKF innovation
  等内部量。它只能拒绝明显动态，不能证明剩余 specific force 一定是重力。
- 基线执行有界三轴 ZARU，但不在线写入永久标定。恒定慢速绕重力轴旋转和 gyro bias
  仅靠这两颗 MEMS 不可区分；因此内部学习被限制为逐轴 `+/-0.5 dps` 和三维向量
  `0.1 dps/s`，不能
  把“已完成 ZARU”解释为已经证明载体绝对静止。
- 双 gyro 1v1 软不一致且不能裁决时，保持切换前选中 lane 的连续输出并标记
  `AMBIGUOUS`；重力 innovation 不能裁决沿重力轴的软故障。
- 温度变化率门、clipping 去抖、运行期配置巡检和故障自动恢复属于后续保护工作，不是当前 FIFO、
  时间戳、MEKF 或 1600 Hz 发布链的输入条件。

## 7. 几何与标定

两颗加计封装中心距按当前 STEP 数据为 7.1 mm。两颗器件均按
`ROTATION_PITCH_180` 转入 FRD 后，使用：

```text
r_ICM_to_BMI    = [0, -0.0071, 0] m
midpoint_to_BMI = [0, -0.00355, 0] m
midpoint_to_ICM = [0, +0.00355, 0] m
```

同一刚体上各点姿态相同，参考点只影响加速度杆臂补偿。比较或融合两颗 accel 时，
统一把 specific force 归算到双 IMU 中点。封装中心只是敏感中心近似；内部 die 的
精确 Z 差没有数据时取 0，并保留几何不确定度。

可在现有条件下完成：手动六面 accel 标定、两 IMU 相对安装矩阵、静置温度零偏曲线、
长时间静置噪声统计。没有已知角速度参考时不能做绝对 gyro scale 标定；使用厂商
标称 scale，只能通过两 lane 互相比较检查相对比例。

## 8. 当前实现与实板结果

| 项目 | 当前状态 |
|---|---|
| MEKF、预积分、FIFO、clock-sync、HI91 和 USB stream 主机测试 | 十二组全部通过 |
| Debug/Release/RelWithDebInfo ARM 固件 | 无 warning 编译并链接通过 |
| SPI1/SPI4 DMA、D2 SRAM staging、I-cache on / D-cache off | 已实现并上板运行 |
| ICM hi-res FIFO、timestamp affine、DMA 和 manager | 已板测，1 s 代表计数约 1610-1612/3219-3223 Hz |
| BMI accel/gyro FIFO、sensor-time/capture ring、DMA 和 manager | 已板测，1 s 代表计数约 1608-1612/1997-2000 Hz |
| `dual_imu_process()` | 已消费异步 FIFO batch/frame，不再逐事件阻塞读寄存器 |
| MEKF gravity gauge | 姿态/bias gain 切平面投影；高噪声不可观方向回归通过，实板 bias 不再拉飞 |
| 内部静止/ZARU | 实板 `stationary_confirmed=1`，两 lane 易失 bias trim 收敛 |
| lane 故障切换 | 主机 yaw/tilt alignment 回归通过；调试停机注入后 BMI fail-close 并切到 ICM |
| 400 Hz MEKF | 实测 399-401 Hz |
| 1600 Hz fast quaternion/Euler | 1 s 代表计数 1598-1602 Hz，正常时有效率相同 |
| TIM5_CH3 625 us 发布调度 | 正常时 miss/drop=0，发布延迟峰值小于 20 us |
| USB HI91 82-byte/1600 Hz | 10.016 s 共 16027 帧，1600.040 Hz；全字段 valid，CRC/resync/drop/deadline 均为 0 |
| 2026-07-15 CubeMX 再生成 | 软件审计、十二组测试、三个 ARM build 和 DMA map 通过；新固件上板 USB 冒烟待做 |
| 时钟链 | 启动 timeline reset=1（预期）；稳态 clock reset/discontinuity/causal error=0 |
| 逐台标定与四条绝对 pipeline delay | 未完成；当前 calibration 未加载、delay 宏为 0 |
| 最终 2 h 数据完整性 | 未验证 |

传感器表中的 ODR 是寄存器名义值，1 s 状态行只是带量化的代表计数，不是精确 ODR
测量。affine clock model 用来吸收 sensor clock 与 TIM5 的斜率差，不能把短窗计数
反写成寄存器配置。当前板上已证明持续 400 Hz MEKF、1600 Hz 有效 fast 输出、内部
ZARU、故障 fail-close 和 USB 满负载输出；CubeMX 再生成的软件回归已经通过。剩余工作
不再是提高频率，而是再生成固件的短时上板冒烟、逐台标定、pipeline delay 标定和
延期的 2 h 长跑。

## 9. 当前可执行验收

可以完成的验收：

1. Debug/Release 固件无 warning 编译，全部主机测试通过。
2. 上电回读 ODR、BW、FSR、FIFO、INT map 和 timestamp 配置，并与第 2、3 节一致。
3. 板上统计四流实际速率、FIFO overflow/parse/DMA error、时钟 reset/discontinuity/
   causal error、MEKF rate、fast valid rate、publish miss/drop 和峰值延迟。
4. 静置连续 2 h：上述 error/drop 保持 0，BMI 24-bit、ICM 16-bit 和 TIM5 32-bit 的
   正常回卷成功 unwrap，映射时间严格单调。这一项尚未完成。
5. 手动六面静置检查四元数和预测重力方向；`+/-X` 面只检查 `euler_singular`，不检查
   奇异点的 Euler roll/yaw。远离奇异区手动绕三轴检查 FRD/ZYX 符号和四元数连续性。
6. 记录热身过程、静置 roll/pitch 稳定性和相对 yaw 漂移；逐台标定未加载时只记录，
   不将结果扩展为精度指标。
7. USB 主机置 DTR 后连续检查 82-byte 对齐、CRC、1600 Hz 帧率、timestamp 单调性、
   pressure/mag 零填、status drop/deadline；此前单会话 10.016 s 短测已满足。

当前没有已知角度/角速度基准，不能宣称：绝对 yaw 正确、绝对 gyro scale 已标定、
绝对 filter/group delay 已知、动态相位满足某个闭环指标，或持续线加速度下的
roll/pitch 是绝对真值。

## 10. 剩余工作

1. 目标板 USB 数据口重新枚举后，刷入 2026-07-15 再生成的 Release，并重复短时
   HI91 CRC/1600 Hz/TIM5 miss-drop 冒烟测试。
2. 加载并验证逐台 accel/gyro/temperature 标定。
3. 标定四条 pipeline delay；当前宏全部为 0，不宣称绝对动态相位。
4. 做受控六面、三轴正转及接近 Euler 奇异区测试，确认 FRD/Hamilton/ZYX 符号。
5. 按用户安排稍后执行 2 h 连续长跑并保存完整计数器结果。此前约 15 分钟运行被主动
   停止且期间板子被移动，不能算作 2 h 通过；运动时短暂 invalid 也不是静态链路结论。
6. 保护功能另行实现，不改变本文件已经定稿的 ODR、FIFO、时间戳、MEKF 和发布频率。

## 11. 官方依据

- [Bosch BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [Bosch BMI08x Data Synchronization](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mis-an006.pdf)
- [Bosch BMI08x FIFO](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mis-an005.pdf)
- [Bosch BMI08x SensorAPI](https://github.com/boschsensortec/BMI08x_SensorAPI)
- [TDK ICM-45686 product, datasheet and AN-000478 user guide](https://www.invensense.tdk.com/en-us/products/6-axis/icm-45686)
- [TDK ICM45686 official driver](https://github.com/tdk-invn-oss/motion.arduino.ICM45686)
