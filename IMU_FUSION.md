# DM-FC01 双 IMU 解算与时间戳规范

本文定义最终算法和接口语义。最终频率、引脚、当前实现状态和开发顺序以
[README.md](README.md) 为准。

## 1. 适用边界

运行时输入只有：

- BMI088 accel + gyro。
- ICM45686 accel + gyro。

算法不依赖其他信号。因此必须满足：

- roll/pitch 在加计主要反映重力时可观。
- yaw 只表示相对旋转，初值为 0，长期漂移不可消除。
- 持续线加速度与倾斜、恒定慢速绕重力轴旋转与 gyro bias，不能仅靠这两颗 MEMS
  完全区分。
- 两 lane 软不一致而内部证据不足时保持 `AMBIGUOUS`，不能伪造故障隔离结论。

## 2. 坐标、姿态与时间约定

- 标定后机体系为 FRD：X forward、Y right、Z down。
- 世界系为 NED。
- 四元数顺序 `[w, x, y, z]`，表示 body 向量旋转到 NED。
- 四元数乘法采用 Hamilton convention。
- 欧拉角为 ZYX yaw-pitch-roll 派生结果，输出顺序 `[roll, pitch, yaw]`。
- 原始样本和姿态 epoch 全部使用 TIM5 64-bit 微秒时间。
- 四元数是主输出；欧拉角接近 pitch `+/-90 deg` 时退化。

任何测试至少要手动绕 FRD 三轴做正向转动，检查 gyro 符号、四元数方向和 ZYX Euler
符号一致。仅有主机数学测试不能发现所有安装方向错误。

## 3. 定稿频率

| 项目 | 名义配置值 | 时间间隔/窗内样本 |
|---|---:|---|
| BMI accel | 1600 Hz, OSR4, +/-24 g | 625 us；每窗约 4 样本 |
| BMI gyro | 2000 Hz, 230 Hz BW, +/-2000 dps | 500 us；每窗约 5 样本 |
| ICM accel | 1600 Hz, ODR/16, hi-res +/-32 g | 625 us；每窗约 4 样本 |
| ICM gyro | 3200 Hz, ODR/16, hi-res +/-4000 dps | 312.5 us；每窗约 8 样本 |
| MEKF/selector | 400 Hz | 2500 us 公共窗 |
| fast quaternion/Euler | 1600 Hz | 625 us 固定输出网格 |
| internal gyro integration | lane native | 全部 2000/3200 Hz 样本参与积分；control API 只返回 latest snapshot |

MEKF 频率和姿态发布频率是两件事。400 Hz MEKF 完成重力校正、bias/covariance 更新
和 lane 选择；1600 Hz fast predictor 只把最新校正姿态用后续 gyro 积分到当前输出
epoch，不制造新的重力信息。

## 4. 四条采样流

系统始终保留四条独立流：

```text
BMI accel 1600 Hz -> FIFO + sensor-time --------+
BMI gyro  2000 Hz -> FIFO + PA1 capture --------+-> BMI lane

ICM accel 1600 Hz -> 20-byte FIFO timestamp ----+
ICM gyro  3200 Hz -> 20-byte FIFO timestamp ----+-> ICM lane
```

每个样本必须携带：

- `timestamp_us`：映射到 TIM5 后的样本 epoch。
- `sequence`：该物理流独立递增。
- `valid` 和数据完整性标志。
- 温度和 source。

FIFO watermark 只是“应读取一批数据”的事件，不是批内每个样本的测量时刻。

## 5. 时间戳链

最终模型为：

```text
t_stream = affine(unwrapped_sensor_counter_or_capture) - relative_delay
```

其中 affine 映射只估计时钟 slope/phase；filter/group delay 是另一项，不能混进 slope。

### 5.1 TIM5

TIM5 为 1 MHz、32-bit 自由计数，通过 update IRQ 扩展到 64 bit。PA0/PA1 使用
TIM5_CH1/CH2 硬件 capture；PE4 只能在 EXTI ISR 中软件读取 TIM5，因此时间质量低于
硬件 capture。优先级固定为 TIM5 1、PE4 EXTI 2、SPI RX/TX DMA 3/4。
M7 I-cache 已启用，D-cache 明确关闭；DMA buffer 尚无 cache clean/invalidate 协议，
因此当前配置不能打开 D-cache。

### 5.2 BMI accel sensor-time

BMI accel sensor-time：

- 24 bit。
- 1 tick = `39.0625 us`。
- 约 655.36 s 回卷一次。
- 作为 FIFO 控制帧出现，不是每个 accel frame 自带时间戳。

PA0 最终是 INT1 FIFO WTM=28 bytes，对应 4 个 7-byte accel 数据帧。只有解析结果恰好
为 4 个有效帧、28 个数据字节且没有 skip/drop/truncate 时，该批次才可建立锚点。
1600 Hz 下每个 accel 样本名义相隔 16 sensor ticks。

首个合格批次从 sensor-time 建立采样 phase，并用 PA0/TIM5_CH1 硬捕获时刻观察
sensor clock 到 TIM5 的 affine 映射。后续正式 watermark sample tick 直接定义为
`last_sample_tick + 4 * 16`，不能用 DMA 完成时读取的 sensor-time 快照覆盖连续 frame 序列。
批内四帧按 16 ticks oldest-first 回填 timestamp。PA0、FIFO request 和 DMA complete
均不直接作为四个样本的测量时刻。结构无效批次不发布并重建 timeline；当前板启动
建立 phase 时 `timeline reset=1`，稳态不增长，clock reset 和 causal failure 均为 0。

### 5.3 BMI gyro capture

BMI gyro 没有与 accel 等价的完整 FIFO sensor-time。PA1/INT3 映射逐样本 DRDY，
TIM5_CH2 ISR 将每个 2000 Hz capture 的 `(timestamp, sequence)` 直接写入 ring；gyro
FIFO 仅用于吸收 DMA 排队，stream/XYZ 保持启用但关闭 watermark interrupt。

启动边界在 gyro 稳定 NORMAL 后建立：FIFO 保持 BYPASS，capture ring 等待并消费一个
真实 DRDY seed，随后在下一边沿前切到 STREAM；配置事务内若出现边沿则回到 BYPASS
重试。因此 STREAM 第一个 frame 与 ring 第一个待消费 capture 对齐。

运行时读取 FIFO frame count N 和 oldest-first 的 N 个 frame，再原子取 ring 中最老 N 个
capture；STATUS/DATA 期间到来的边沿留在队尾。数量、sequence/timestamp、truncate、
overrun、capture overflow 或已启动 FIFO_DATA DMA 的任一不确定失败都会整批丢弃并锁存
gyro timing fault，停止该 lane 后续 gyro DMA；不能等待未来边沿、自动猜测恢复，或按名义
500 us 间隔伪造 epoch。

### 5.4 ICM FIFO timestamp

ICM 普通 20-byte high-resolution FIFO 帧包含 16-bit timestamp，1 us 分辨率下正常
每 `65.536 ms` 回卷。最终配置为 `TMST_DELTA_EN=0` 和 `FIFO_TMST_FSYNC_EN=1`。前一项确保字段是滚动
counter 而不是 trigger delta；后一项在 FSYNC 关闭时插入普通 timestamp。当前上电
配置显式回读这两项以及 `SMC_CONTROL_0` 的 timestamp enable；运行期再由后续的非阻塞
configuration watchdog 巡检。处理顺序：

1. 检查 header、INVALID 字段、FSYNC tag、overflow 和 frame size。
2. 解卷 16-bit timestamp。
3. 用干净且 `count >= 2` 的批次建立 PE4/TIM5 anchor。
4. 至少 4 个 anchor 被接受后，`clock_sync_valid` 才能为 true。
5. 用 affine 模型映射每个正常 timestamp。

gyro 3200 Hz、accel 1600 Hz 时，没有新 accel 样本的 gyro 帧会把 accel 字段标为
INVALID；parser 只提交实际 valid 的 accel，不复制上一帧。ICM 使用 watermark equal
模式，PE4 边沿的所有权固定属于 oldest-first FIFO 中索引 `WM-1` 的包，即 WM=2 时的
第 2 帧。DMA 启动前又有新包入 FIFO 不改变边沿所有权，因此不能把批次最后一帧硬编码
成锚点，也不能因 `count > 2` 拒绝本来有效的第 2 帧锚点。当前实板时钟链的
reset/discontinuity/causal error 均为 0。

### 5.5 未使用的 ICM pin9

最终保持 ICM pin9 为 INT2 功能，PE3 为 MCU 输入但不映射中断；FSYNC 和 CLKIN 均
关闭。两颗 MEMS 自由运行，所有样本统一映射到 TIM5，不存在同步脉冲输入或脉冲触发
一次转换的路径。

## 6. 公共窗预积分

两个 lane 使用相同的 `[t_k, t_k + 2500 us]` 窗。窗边界处按相邻样本时间做分段
插值，所有 gyro 子区间都参与积分。实现保留：

- 总 delta angle 和 delta quaternion。
- 前半窗、后半窗 delta angle。
- 窗首/窗尾/均值 gyro。
- `E[omega * omega^T]` 二阶矩。
- accel 的时间加权均值。
- coverage、gap、drop、timestamp order 和 queue overflow。

圆锥补偿保留子增量的非交换二阶项。这是有限采样和分段模型下的二阶近似，不称为
连续真实运动的精确积分。缺样不能通过长区间线性插值掩盖；超过每流 `max_gap`
后整窗 propagation invalid。

姿态 epoch 为窗尾，窗均值 accel/gyro 的代表时刻为窗中点。两者相差 1250 us 是
有意定义，不能把它们写成同一个时间戳。

## 7. 双 lane MEKF

每个 lane 的误差状态：

```text
x_error = [delta_theta_x, delta_theta_y, delta_theta_z,
           delta_bg_x,    delta_bg_y,    delta_bg_z]
```

名义姿态使用 quaternion；gyro bias 为本 lane 独立状态。每窗执行：

1. 用前半窗 delta angle 传播到窗中。
2. 在窗中进行门控后的重力方向更新。
3. 用后半窗传播到窗尾。
4. 更新 covariance、innovation、health 和 selector 输入。

selector 的双 lane delta-angle 残差使用窗起始、任何在线 bias 更新之前的 bias 快照，
防止某一 lane 的 bias 更新先把故障残差吸收掉。

加计更新在重力切平面上是 rank-2，只约束 roll/pitch 方向，不能观 yaw。Joseph 形式
协方差更新和数值条件检查继续保留。姿态误差和 gyro-bias 两个 Kalman gain block 都
先投影到 body-frame 重力切平面，修正与 Joseph 更新复用同一投影后 gain，禁止
covariance cross-correlation 把加计噪声注入 yaw 或重力轴 bias。该投影只保留 gauge，
不增加 yaw 观测。

### 7.1 启动对准

当前实现等待两个 lane 都形成完整的首个 2.5 ms 公共窗，再用有效 accel 窗均值初始化
roll/pitch，yaw 定义为 0，gyro bias 以 0 起步。若本 lane accel 无效，可由另一条健康
accel lane 辅助初始化。当前板没有加载逐台 accel/gyro/temperature 标定，因此该启动
姿态和静置漂移只用于链路验证，不代表最终精度；在逐台标定加载前不承诺 yaw 漂移。

### 7.2 加计门控

重力更新只使用内部证据：

- MEKF 内部的 `|norm(a)-g|` 和 innovation/NIS。
- 两加计平移到公共参考点后的差异及其 variance scale。
- gyro rate 和 angular acceleration。
- sample coverage、gap、timestamp 和 lane health。

两个加计同时受到相同线加速度时仍可能一致，所以这些门只能拒绝明显动态，不能证明
剩余向量一定是重力。被门控时继续 gyro propagation，并增大不确定度。

### 7.3 Bias 与内部 ZARU 边界

两 lane 以 0 bias 启动。内部静止候选同时要求两 lane 数据完整、角速度低于上限、单窗
振动方差和驻留期跨窗 gyro/accel 方差受限、加速度模长接近 g、双加计一致且重力方向
稳定。统计先预热 40 窗，驻留 gyro 均值还必须逐 lane、逐轴不超过 `0.5 dps`；连续
2 s 后执行三轴 ZARU，不依赖其他输入。当前实测 temporal gyro 总 RMS 门分别为 BMI
`0.8 dps`、ICM `0.25 dps`，accel 总 RMS 门为 `0.10 m/s^2`。ZARU 对当前 calibration
pipeline 输出的 target 逐轴限制为 `+/-0.5 dps`，bias correction 三维向量的 slew
限制为 `0.1 dps/s`，收敛误差连续小于 `0.05 dps` 后才置 lane calibrated。

恒定慢速绕重力轴旋转和重力轴 gyro bias 仍然不可区分；上述限幅只封顶错误学习的
伤害，不能创造观测性。当前尚无温度变化率门，且未加载逐台温度标定，因此不承诺最终
yaw 漂移指标。`lane_calibrated` 只表示本次上电的易失 bias trim 收敛，不代表 scale、
安装或温度标定完成，也不把一次内部 ZARU 完成当作永久标定。

## 8. Selector 与完整性

selector 使用两个 gyro lane 的公共窗 delta angle、各自 covariance、硬故障和持续
统计。正常时选择一个 lane，不平均。

只有两票时，bus/FIFO/timestamp 等可归因硬错误可以隔离对应 lane；软残差只有 1v1，
不足以知道哪一颗坏。此时保持切换前选中 lane 的连续输出，并输出
`AMBIGUOUS + integrity_degraded`，不能声称当前 lane 健康。gravity innovation 只能
作为低动态下的软证据，不能裁决沿重力轴的软 bias。clipping 去抖、运行期配置巡检和
自动恢复属于后续保护工作，不改变这里的双 lane 数学边界。

切换瞬间先构造完整 alignment 保持输出 quaternion 连续，再将 alignment 分解为绕
world-Z 的 yaw twist 和可观 tilt swing。只有 tilt 以 `1 rad/s` 衰减；yaw twist 永久
保留，因为本系统没有航向观测。`alignment_blend_active=false` 仅表示 tilt 已收敛，
不表示两个 lane 的 yaw gauge 已被强制统一。

## 9. 1600 Hz fast attitude predictor

400 Hz selector 输出的连续四元数作为 anchor。fast predictor 保存 anchor epoch 之后
经过当前 calibration pipeline 的 gyro ring，并在每个 625 us TIM5 输出 epoch 执行：

1. 取 selector 当前 lane 及其最新 bias。
2. 从 anchor epoch 开始，按每个真实 gyro timestamp 积分全部子增量。
3. 对输出边界做时间比例切分，不把 FIFO 到达时刻当测量时刻。
4. 若输出 epoch 晚于已收到的最新 gyro epoch，用最新去偏角速度作短时零阶保持外推。
5. 生成 `q_fast`；再派生 ZYX Euler。
6. `latest_gyro_timestamp_us` 只指已经收到且不晚于输出 epoch 的最新有效样本；
   `prediction_horizon_us` 上限固定为 `3000 us`。
7. 只有选中 lane 从 anchor 到输出 epoch 的 timestamp 链或样本覆盖无效时才输出
   invalid；未选 lane 的 clock/gap 故障不得停止输出。

每个新 gyro 样本推进选中 lane 的 replay cache；1600 Hz compare tick 的常见路径直接
使用缓存结果并做有界 ZOH，不再每 tick 重放整个历史。新 400 Hz anchor 到达时只从新
anchor epoch 重放后续 ring 样本，以免 correction 和旧 predictor 增量重复计算。
selector 切 lane 时，若新 lane 有覆盖 anchor 到输出 epoch 的 gyro 历史和上述保 yaw
quaternion alignment，则继续发布连续的 degraded 输出；只有缺少共同时间覆盖才
invalid。source/integrity 必须即时更新。

最终接口至少包含：

```c
typedef struct {
    float quaternion[4];
    float euler_rad[3];
    float gyro_rate_rad_s[3];
    uint64_t anchor_timestamp_us;
    uint64_t latest_gyro_timestamp_us;
    uint64_t output_timestamp_us;
    uint64_t publish_timestamp_us;
    uint32_t sequence;
    uint32_t prediction_horizon_us;
    uint32_t publish_latency_us;
    uint32_t integrity_flags;
    imu_source_t selected_source;
    bool predicted;
    bool degraded;
    bool deadline_miss;
    bool euler_singular;
    bool valid;
} dual_imu_attitude_output_t;
```

`gyro_rate_rad_s` 是生成该 frame 时 predictor 实际使用的去偏角速度快照，不能在消费
历史 frame 时再用“当前最新 gyro”替换；否则 USB 队列短暂积压会混合不同 epoch。

`valid` 只表示数值、时间链和样本覆盖有效；它不表示逐台标定已加载、ZARU 已收敛或
姿态精度已经达标。

固定输出率为 1600 Hz。内部可以在每个 2000/3200 Hz gyro 样本到达时更新 accumulator，
但对外姿态 epoch 仍落在固定 625 us 网格，避免 lane 切换导致接口频率变化。ICM
WM=2 只定义阈值：无积压且及时读空时名义约 2 帧/批、1600 IRQ/s，实际 DMA 批次
必须按 FIFO count 读取并保留每帧 timestamp。

每个 tick 必须独立 release，`DUAL_IMU_ATTITUDE_PUBLISH_DEADLINE_US=250`。若实际
`publish_timestamp_us > output_timestamp_us + 250 us`，该帧置 `deadline_miss` 和
invalid；禁止成批晚发或重复上一姿态。`publish_latency_us` 记录实际发布延迟。

## 10. Euler 输出语义

Euler 输出必须同时满足：

- 角度单位 rad。
- roll/pitch/yaw 顺序固定。
- yaw 默认包角到 `[-pi, pi]`；需要连续 yaw 的调用方另行 unwrap。
- `|pitch| >= 85 deg` 时设置 `euler_singular`。
- yaw 始终标为 relative，不提供 absolute heading 标志。
- output timestamp 是 fast predictor 的 epoch，不是计算完成时刻。

“1600 Hz Euler”只表示发布频率，不表示绝对观测带宽为 1600 Hz。动态信息仍受 gyro
200/230 Hz 名义低通、滤波群延迟、`prediction_horizon_us`、发布延迟和当前未标定的
绝对 pipeline delay 限制。

## 11. 当前实现与实板结果

当前已有：

- 四流 buffer、公共窗预积分、二阶圆锥项。
- 双 6-state MEKF、gravity gauge 投影、selector 和保 yaw 切换 alignment。
- 1600 Hz fast predictor、`dual_imu_get_attitude()` latest copy-out API，以及用于 USB
  顺序消费每个 TIM5 epoch 的 SPSC 发布 ring。
- predictor 的真实时间戳梯形积分、二阶圆锥项、replay cache、重锚重放、短时 ZOH、
  lane 切换连续、horizon/deadline/stale/Euler 奇异语义及对应主机测试。
- TIM5 64-bit、BMI capture、sensor clock affine helper，以及 CH3 的 625 us compare 调度。
- ICM hi-res FIFO parser、异步 SPI4 DMA 状态机和 manager 逐帧接入。
- BMI accel/gyro FIFO、异步 SPI1 DMA、sensor-time/capture ring 时间重建和 manager 接入。
- D2 SRAM DMA staging，I-cache enabled、D-cache disabled；完整 ARM Debug/Release 固件
  在严格 warning 下链接通过。
- DMA staging 的 2 KiB 地址区由项目自有 `linker/dual_imu_h743.ld` 固定保留，并由
  CMake 选择检查和 linker 地址断言保护，不依赖 CubeMX 默认 linker script。
- 十二组主机单元测试，包括高噪声 gauge、保 yaw failover、静止拒绝原因、有界 ZARU、
  HI91 CRC/布局和 USB 队列/会话语义。
- 当前实板 1 s 代表计数约为 `1608-1612/1997-2000/1610-1612/3219-3223 Hz`，名义
  配置为 `1600/2000/1600/3200 Hz`；短窗计数同时包含时钟容差和量化。
- MEKF `399-401 Hz`；fast quaternion/Euler 1 s 代表计数 `1598-1602 Hz`，正常时
  有效率相同。compare miss/drop 为 0，发布延迟峰值小于 `20 us`。
- BMI accel 启动 phase 建立产生一次预期 timeline reset；稳态 clock reset、
  discontinuity 和 causal error 为 0。
- 实板静置已达到并可重新进入 `stationary_confirmed`，两 lane 易失 bias trim 均收敛；
  稳定区间 temporal gyro 总 RMS 的典型值约为 BMI `0.52-0.56 dps`、ICM
  `0.08-0.15 dps`。噪声尖峰会保守退出静止判据，但不清除已收敛 trim。
- 调试停机注入使 BMI gyro 时间链 fail-close、selector 切到 ICM；健康 lane 恢复
  1600 Hz 有效输出。自动重建故障 lane 不在本阶段范围内。
- USB CDC 使用固定 82-byte HI91 布局和 DTR 会话门控；单会话 10.016 s 板测收到
  16027 帧，`1600.040 Hz`，attitude/gyro/accel 全部 valid，CRC/resync/transport
  drop/deadline miss 均为 0。完整字段和状态位定义
  见 README 5.2。
- 2026-07-15 CubeMX 再生成后，十二组主机测试、Debug/Release/RelWithDebInfo 和
  `.imu_dma` 地址断言重新通过；再生成固件的短时 USB 上板冒烟因数据口未枚举而待做。

当前仍未完成或未验证：

- 当前板未加载逐台 accel/gyro/temperature 标定，静置姿态与漂移不能作为精度指标。
- 四个 pipeline delay 宏均为 0，绝对动态相位未标定。
- 2 h 连续长跑尚未完成。
- 此前约 15 分钟长跑按用户要求停止且期间板子被移动，不能作为 2 h 通过；动态阶段
  短暂 invalid 不据此判作静态链路故障。
- ZARU 的温度变化率门、clipping 去抖、运行期 configuration watchdog 和故障自动恢复
  属于后续保护工作。

因此当前已经达到目标 FIFO 数据率、400 Hz MEKF 和 1600 Hz 有效姿态发布；这项结论
只覆盖链路频率、时间戳完整性和调度，不扩展为已达到最终姿态精度或长期可靠性指标。

## 12. 可执行验收

当前验收包含：

1. 全固件 Debug/Release 编译和所有主机测试通过。
2. 配置 readback 与第 3 节完全一致。
3. 板上统计四流 ODR、FIFO overflow/parse/DMA error、时钟 reset/discontinuity/causal
   error、MEKF rate、fast valid rate、publish miss/drop 和峰值延迟。
4. 静置 2 h：上述 error/drop 保持 0，BMI 24-bit、ICM 16-bit 和 TIM5 32-bit 正常回卷
   成功 unwrap，映射时间严格单调。这一项尚未完成。
5. 初始化及两条时钟链有效后，scheduled/valid rate 的 1 s 代表计数为
   `1598-1602 Hz`，长期目标为 1600 Hz；
   `publish_latency_us <= 250`，deadline miss/drop 为 0；当前短时板测已满足。
6. 手动六面静置检查 quaternion/预测重力；`+/-X` 面检查 `euler_singular`，不验收
   奇异点的 Euler roll/yaw；远离奇异区手动三轴正转检查 FRD/ZYX 符号。
7. 记录热身过程、静置 roll/pitch 稳定性和相对 yaw 漂移；逐台标定未加载时只记录，
   不作为精度验收。
8. 已有主机测试覆盖 fast predictor 常速/ramp、非交换旋转、圆锥项、ring 回绕、重锚
   不重复积分、gap/sequence/timestamp 错误、未来样本隔离、lane 切换连续及 Euler
   奇异标志，以及 BMI FIFO/capture 启动边界、generation 失效、DMA 异常和有界内部 ZARU。

验收结果不能扩展为没有测过的声明：不报告绝对动态角度误差、绝对 group delay、
绝对 yaw 或持续线加速度下的绝对 roll/pitch。
