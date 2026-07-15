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
gyro timing fault，停止该 lane 后续 gyro DMA；不能等待未来边沿或按名义 500 us 间隔
伪造 epoch。锁存 20 ms 后可执行受控重建：原子清 TIM5_CH2/capture，保持 NORMAL，
BYPASS 后用真实 DRDY seed 对齐 STREAM，再只回读 gyro 的 chip ID、量程/ODR/电源、
INT3 和 FIFO。SPI 暂忙 1 ms 后再试，真实失败退避 1 s；首个新可信 batch 前保持输出
无效。该流程不是根据名义 ODR 猜测恢复，也不覆盖 accel、ICM 或广义总线故障。

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

两 lane 以 0 bias 启动，但 gravity 更新从首个有效 accel 窗开始就通过姿态-bias
cross-covariance 学习重力切平面内可观的 gyro bias；这个过程不要求
`lane_calibrated=true`。重力平行方向仍不可观，不能由 accel 更新伪造出来。

内部静止候选允许双健康 lane 或单健康 lane。它要求样本和滤波器状态有效、瞬时角速度
低于安全上限、单窗与驻留期 gyro/accel 方差受限、加速度模长接近 g，并在双 lane 时
检查两加计一致性。统计预热 40 窗后，每个参与 lane 使用当前已 seed MEKF 的三维 bias：

```text
r_lane = dwell_mean(gyro_calibrated) - current_mekf_bias
norm(r_lane) <= 0.5 dps
```

该门是完整三维向量范数，不是 raw gyro 均值的逐轴判断。默认双 lane 连续通过 2 s、
单 lane 连续通过 4 s 后才确认静止；单 lane 确认时只对被跟踪的健康 lane 执行 ZARU。
当前 temporal gyro 总 RMS 门分别为 BMI `0.8 dps`、ICM `0.25 dps`，accel 总 RMS 门为
`0.10 m/s^2`。

确认静止后只运行普通三轴 ZARU：bias correction 三维向量的 slew 上限为
`0.1 dps/s`。ZARU target 与更新后 bias 的距离连续 40 窗小于 `0.05 dps` 才置
`lane_calibrated`；已校准 lane 在确认静止期间连续 40 窗 ZARU 拒绝或距离超过
`0.20 dps` 时撤销该状态。不存在启动期放宽的 bias 学习路径。

物理一致回放固定验证三条可观测性边界：

- `gyro=[1 dps,0,0]` 且 accel 随真实 roll 旋转时，不能进入 `STATIC`、不能执行 ZARU，
  也不能把真实转动学成 X bias。
- accel 固定水平而 `gyro=[1 dps,0,0]` 时，gravity 更新先估计可观 X bias；残差进入
  `0.5 dps` 门后，再由普通 ZARU 完成收敛。
- accel 固定水平而 `gyro=[0,0,1 dps]` 时，重力轴 bias 不可观，必须持续拒绝静止和
  ZARU。

恒定低于静止门限的绕重力轴慢转和重力轴 gyro bias 仍然不可区分；限幅和驻留只能控制
风险，不能创造观测性。`lane_calibrated` 只表示本次上电的易失 bias trim 收敛，不代表
scale、安装或温度标定完成，也不把一次 ZARU 完成当作永久标定。

### 7.4 被动温度补偿安全边界

校准结构已支持 accel/gyro 的 `c0..c3` 温度多项式、有效温区、时间因果、最大 2 s
温度年龄和温度变化率门；默认最大变化率为 `5 degC/s`。两条流各自保存 accepted
temperature，BMI accel 先消费较新温度后，不会把随后到达但对 gyro 样本仍因果的较旧
温度误判为 noncausal。被拒绝的温度不更新 accepted history。

ARM 构建固定 `IMU_CALIBRATION_PRODUCTION_BUILD=1` 且
`IMU_CALIBRATION_HIGH_ORDER_TEMPERATURE_COMPENSATION_BUILD_ENABLE=0`，CMake 和预处理器
双重拒绝 production+high-order。宏为 0 时 runtime enable BSS、温度历史、高阶选择和
slew 代码均被编译剔除，静态 `c0` 与 matrix 仍保持原生产运算顺序。当前固件没有 enable
调用，也不存在 heater、PWM 或温度闭环。

隔离的 qualification build 中，目标 correction 在最终输出坐标计算为 `M*bias(T)`，
首次 applied correction 为 `M*c0`。full、clamp、invalid、stale、noncausal、rate、disable
与恢复都只改变 target，再按三维向量范数向 target 限速；accel/gyro 独立配置速率，单步
`dt` 最多 10 ms。默认 accel 为 `0.0980665 m/s^3`，gyro 为
`0.00174532925 rad/s^2`。异常往返、长 gap、向量范数和双流交错已有普通及 sanitizer
测试。正式启用前仍必须取得逐台热箱系数、完成带 CRC/版本的掉电持久化并做实物热循环
验收；这些门禁尚未完成。

该数值路径与成熟飞控的被动补偿方法一致：[PX4 thermal calibration](https://docs.px4.io/main/en/advanced_config/sensor_thermal_calibration.html)
对 accel/gyro offset 使用三阶温度多项式和 `TREF/TMIN/TMAX`，并要求冷浸、温区覆盖和
曲线检查；[ArduPilot IMU temperature calibration](https://ardupilot.org/copter/docs/common-imutempcal.html)
同样按每颗 IMU 独立采集、保存并离线核验。它们支持的是逐台、全温区验证后的被动数值
修正，不是用未知系数上线，也不要求启动 heater 或温度闭环。

### 7.5 陀螺静态 g 敏感度资格路径

资格代码支持
`omega_out = omega_temperature_matrix - K_ga * f_body`。`f_body` 是同一颗 IMU 完成轴向
旋转、静态/温度矩阵校准后的 body/output-frame specific force；`K_ga` 的单位为
`(rad/s)/(m/s^2)`，行对应 gyro 输出轴，列对应 accel body 轴。该修正位于普通 gyro
校准之后、motion guard、buffer、predictor 和 estimator 之前。

每个 source 保存独立的短因果 accel 历史。查询只选 `accel_timestamp <= gyro_timestamp`
的最新同源样本；未来样本不会遮住较旧的因果样本。invalid、时间不可信、gap 或 accel
saturation 会清除旧证据并建立时间屏障，非零矩阵启用时若没有未饱和且足够新的证据，
对应 gyro 样本 fail closed，不进入姿态链。零矩阵路径在读取 accel 前短路并逐位保持 gyro。

该路径受独立的 `IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE` 保护。ARM production 固定为
`0`，预处理器和 CMake 都拒绝 production+g-sensitivity，加载 calibration 也不会自动启用；
只有 production=0 的隔离资格构建可以在首个流样本前显式 enable。当前板没有 `K_ga`
系数，两个矩阵均为零，从未启用。

单一平放静止记录只有一个固定的 specific-force 向量，`K_ga` 与常值 gyro bias 不可辨识。
[TDK AN-000265](https://d17t6iyxenbwp1.cloudfront.net/s3fs-public/2026-06/AN-000265%20TDK%20IMU%20Calibration%20Application%20Note%20v1.1.pdf)
明确采用 `gyro_ideal = S_g*gyro_raw + GS_g*accel_raw + B_g`，其中 `GS_g` 是 3x3
g-sensitivity 矩阵；联合求解 21 个 gyro 参数需要带真实角速度的 2-DOF 转台和七个静/动态
姿态。多姿态静置只能在固定其他参数的前提下做初步辨识，不能替代联合标定；冲击/振动
资格还需 rate table 及 shaker 或 centrifuge 覆盖目标方向、幅值和频率，并使用独立留出集。

[BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
给出的 gyro g-sensitivity 上限为 `0.1 dps/g`（刺激频率低于 20 kHz），gyro ZRO 温漂为
`+/-0.015 dps/K`，并提供最高 `+/-24 g` accel 量程以降低强信号 clipping；当前
[ICM-45686 datasheet](https://d17t6iyxenbwp1.cloudfront.net/s3fs-public/2026-04/ds-000577-icm-45686-datasheet.pdf)
给出 `0.0038 dps/sqrt(Hz)` gyro noise density、`+/-0.005 dps/degC` ZRO 温漂以及最高
`+/-4000 dps`/`+/-32 g` 量程。这些器件级边界证明误差机制存在，但不能代替本板逐台
系数。没有联合拟合和留出验证前，本节只说明代码安全边界；静态线性矩阵也不能重建
clipping 时丢失的角增量，量程事件仍必须 fail closed 并重新捕获姿态。

### 7.6 2026 证据与架构权衡

板上最终保留“双独立 6-state MEKF + ICM preferred selector”。不采用只保留一个
fused output 的固定等权平均路径：按当前 gyro noise density，BMI 为
`0.014 dps/sqrt(Hz)`、ICM 为 `0.0038 dps/sqrt(Hz)`，等权平均噪声约为 ICM 单独使用的
`1.91x`。理想独立白噪声下，逆方差融合的 ICM 权重约 `93.1%`，相对 ICM 单独使用的
理论标准差收益也只有约 `3.49%`。

并行保留 raw/filter lanes 做 diagnostics，同时另算 covariance-weighted output，原则上
可以兼得诊断与小幅降噪；平均本身并不必然破坏诊断。当前不采用它，是因为理论收益很小，
而真实噪声相关性、逐台 bias/scale、相位和安装误差尚未标定，会使这项理想收益进一步
失真并扩大实现验证面。因此 selector 是当前约束下的工程权衡，不是所有多 IMU 系统的
唯一最优结构。两颗 IMU 仍只有两票，只能检测软分歧；没有独立证据时不能可靠归因哪一颗
错误。

这不是本项目独有的保守选择。
[PX4 v1.17.0 EKF2 selector](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/ekf2/EKF2Selector.cpp)
明确在 gyro/accel 数量为 2 时只报告存在故障，数量至少为 3 才用最大累计误差归因坏件；
[ArduPilot EKF lanes](https://ardupilot.org/copter/docs/common-apm-navigation-extended-kalman-filter-overview.html)
也为不同 IMU 运行独立 core，并只输出健康度最好的单一 core。2025 年的
[冗余 IMU 相对安装优化](https://doi.org/10.2514/1.G008473)则以一颗 gyro 传播，并用额外
gyro、星敏感器和残差分布检验提高隔离能力；它说明独立观测能打破歧义，不直接证明
selector 比所有加权融合更优。本板安装已经固定且没有外部姿态真值，不能照搬其结论。

[VQF](https://doi.org/10.1016/j.inffus.2022.10.014) 和
[Mahony 非线性互补滤波](https://doi.org/10.1109/TAC.2008.923738) 是单 IMU 姿态的强基线；
VQF 用于 host replay/shadow 对照，不直接替换板上的完整性架构。它能改善单 lane 的
静态 bias/倾斜估计和扰动门控基线，但同样不能仅凭 6 轴数据创造绝对 yaw 观测、恢复
clipping 时漏掉的转角，或判断两颗互相矛盾的 IMU 中哪一颗正确。
[Markley quaternion averaging](https://ntrs.nasa.gov/citations/20070017872) 说明若比较姿态
平均，必须在四元数几何上处理，不能逐分量平均，但本项目的噪声比和故障目标仍支持
selector。

[Forster IMU preintegration](https://doi.org/10.1109/TRO.2016.2597321) 与
[GTSAM](https://github.com/borglab/gtsam) 适合把 IMU 接入 factor graph；只有加入视觉、
GNSS、encoder、可靠 ZUPT 等外部因子时才增加新约束，单纯把同一双 IMU 历史改写成图
不会恢复绝对 yaw 或漏掉的饱和转角。
[RIANN](https://doi.org/10.3390/ai2030028) 等学习法当前只允许 host shadow 评估，不参与
安全输出或在线 bias 写回。多 IMU 噪声与可观测性依据参考
[Skog inertial sensor arrays](https://doi.org/10.1109/TSP.2016.2560136)：空间分离的 accel
只有在阵列几何和运动激励充分时才增加旋转信息，多个 gyro 本身主要增加冗余。
[Duplex IMU FDI](https://doi.org/10.3390/s21093066) 展示了双源隔离的另一条路线，但其
输入是两套 9-DoF IMU（包含 magnetometer），并使用 UAV 动态模型和两个并行 particle
filters 提供 analytical redundancy；它不能作为两颗裸 6 轴仅靠互差即可归因的依据，反而
说明双源隔离必须引入额外模型或观测假设。

## 8. Selector 与完整性

selector 使用两个 gyro lane 的公共窗 delta angle、各自 covariance、硬故障和持续
统计。正常时选择一个 lane，不平均。

只有两票时，bus/FIFO/timestamp 等可归因硬错误可以隔离对应 lane；软残差只有 1v1，
不足以知道哪一颗坏。此时保持切换前选中 lane 的连续输出，并输出
`AMBIGUOUS + integrity_degraded`，不能声称当前 lane 健康。gravity innovation 只能
作为低动态下的软证据，不能裁决沿重力轴的软 bias。

motion guard 对单 lane gyro 连续 clipping 去抖 3 ms；两 gyro 在 3 ms 内共同 clipping
或任一 accel clipping 形成冲击证据，20 ms 内禁止 saturation hard latch，并暂停 accel
修正 100 ms。饱和样本仍严格拒绝积分。BMI gyro FIFO/capture 采用第 5.3 节受控恢复；
被动温补已有温度变化率门，但高阶项受第 7.4 节生产构建锁保护；静态 g 敏感度资格路径
受第 7.5 节独立构建锁保护；全配置 watchdog 和其他 lane/总线恢复仍未实现。这些保护
不改变双 lane 不可观方向和 1v1 软故障的数学边界。

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

每个新 gyro 样本在主线程推进选中 lane 的 replay cache，并导出只读双缓冲 snapshot；
snapshot 保留最近两个因果端点、anchor/bias、coverage 和完整性状态。1600 Hz TIM5 ISR
固定只读取已发布 snapshot 并做有界 ZOH，不遍历可变历史，也不长时间关闭全局中断。
新 400 Hz anchor 到达时只从新 anchor epoch 重放后续 ring 样本，以免 correction 和旧
predictor 增量重复计算。
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
  lane 切换连续、不可变双缓冲 snapshot、固定复杂度 ISR、horizon/deadline/stale/Euler
  奇异语义及对应主机测试。
- TIM5 64-bit、BMI capture、sensor clock affine helper，以及 CH3 的 625 us compare 调度。
- ICM hi-res FIFO parser、异步 SPI4 DMA 状态机和 manager 逐帧接入。
- BMI accel/gyro FIFO、异步 SPI1 DMA、sensor-time/capture ring 时间重建和 manager 接入。
- D2 SRAM DMA staging，I-cache enabled、D-cache disabled；完整 ARM Debug/Release 固件
  在严格 warning 下链接通过。
- DMA staging 的 2 KiB 地址区由项目自有 `linker/dual_imu_h743.ld` 固定保留，并由
  CMake 选择检查和 linker 地址断言保护，不依赖 CubeMX 默认 linker script。
- 主机测试矩阵包括高噪声 gauge、保 yaw failover、静止拒绝原因、普通有界 ZARU、三类
  可观测性回放、selected-lane numeric fault、单 lane 冲击恢复、HI91 CRC/布局、USB
  队列/无效保留/会话语义、motion guard、HI91 capture 和 static analysis tool。十五组
  普通 host tests 与十四个 C 目标的 ASan/UBSan 回归均通过。
- 当前最终 Release BIN 为 `148384 B`，SHA-256 为
  `93ec06e7684d3c667a33046debf3baa8c265002631e2e9e5aab4020785afb54d`；三种 ARM
  clean-first build 均通过，但不同构建类型的 BIN 大小和哈希不要求相同。
- 当前实板 1 s 代表计数约为 `1608-1612/1997-2000/1610-1612/3219-3223 Hz`，名义
  配置为 `1600/2000/1600/3200 Hz`；短窗计数同时包含时钟容差和量化。
- MEKF `399-401 Hz`；fast quaternion/Euler 1 s 代表计数 `1598-1602 Hz`，正常时
  有效率相同。compare miss/drop 为 0，发布延迟峰值小于 `20 us`。
- BMI accel 启动 phase 建立产生一次预期 timeline reset；稳态 clock reset、
  discontinuity 和 causal error 为 0。
- 实板静置已达到并可重新进入 `stationary_confirmed`，两 lane 易失 bias trim 均收敛；
  稳定区间 temporal gyro 总 RMS 的典型值约为 BMI `0.52-0.56 dps`、ICM
  `0.08-0.15 dps`。噪声尖峰会保守退出静止判据，但不清除已收敛 trim。
- 此前 `852d...` 镜像冷启动时，首个有效 HI91 frame 为 `518 ms`；`ATT_CONV` 在 `527 ms`
  清除，ICM/BMI fault 分别在 `738 ms`/`2928 ms` 最终清除，`STATIC` 在 `4928 ms`
  置位，`WB_CONV` 在 `6133 ms` 清除。这些是该次启动的观测值，不写成所有温度和供电
  条件下的固定时限。
- 此前 `852d...` 镜像 60 s 静置收到 `96006` 帧，平均 `1599.928 Hz`；CRC、resync、drop、
  nonfinite、valid dropout、全零三轴等完整性错误均为 0。末 5 s 均值减首 5 s 均值为
  roll `-0.019 deg`、pitch `-0.004 deg`、yaw `+0.024 deg`。板当前物理安装并非算法 FRD
  水平姿态，这些差值只描述该次静置稳定性，不替代六面安装标定。
- 稳定设备路径为 CDC
  `/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_366437643533-if00` 和
  J-Link `/dev/serial/by-id/usb-SEGGER_J-Link_000069655208-if00`；不得把易变的
  `/dev/ttyACM*` 编号写入脚本或验收记录。
- USB CDC 使用固定 82-byte HI91 布局和 DTR 会话门控；完整字段和状态位定义见
  README 5.2。

当前仍未完成或未验证：

- 当前板未加载逐台 accel/gyro/temperature 标定，静置姿态与漂移不能作为精度指标。
- 四个 pipeline delay 宏均为 0，绝对动态相位未标定。
- 当前 `93ec...` 镜像的冷启动复测及完整 2 h 静置长跑尚待 CDC 重新枚举后从零开始；
  旧 `852d...` 的 1156 s 文件已废弃，不能拼接继承或称为“2 h 通过”。
- 运行期全 configuration watchdog、BMI accel/ICM/广义总线恢复、IWDG，以及高阶温补
  的安全掉电持久化仍未实现。当前不实现 IMU 加热器或温度闭环。

因此当前已经达到目标 FIFO 数据率、400 Hz MEKF 和 1600 Hz 有效姿态发布；这项结论
只覆盖已完成的短时链路频率、时间戳完整性、调度和静置回放，不扩展为已达到最终姿态
精度或 2 h 长期可靠性指标。

## 12. 可执行验收

当前验收包含：

1. 全固件 Debug/Release 编译和所有主机测试通过。
2. 配置 readback 与第 3 节完全一致。
3. 板上统计四流 ODR、FIFO overflow/parse/DMA error、时钟 reset/discontinuity/causal
   error、MEKF rate、fast valid rate、publish miss/drop 和峰值延迟。
4. 静置 2 h：上述 error/drop 保持 0，BMI 24-bit、ICM 16-bit 和 TIM5 32-bit 正常回卷
   成功 unwrap，映射时间严格单调。当前 `93ec...` 的完整采集尚未开始，不是通过状态。
5. 初始化及两条时钟链有效后，scheduled/valid rate 的 1 s 代表计数为
   `1598-1602 Hz`，长期目标为 1600 Hz；
   `publish_latency_us <= 250`，deadline miss/drop 为 0；此前 `852d...` 镜像 60 s
   板测已满足帧率和完整性要求，发布延迟仍按独立诊断计数验收。
6. 手动六面静置检查 quaternion/预测重力；`+/-X` 面检查 `euler_singular`，不验收
   奇异点的 Euler roll/yaw；远离奇异区手动三轴正转检查 FRD/ZYX 符号。
7. 记录热身过程、静置 roll/pitch 稳定性和相对 yaw 漂移；逐台标定未加载时只记录，
   不作为精度验收。
8. 已有主机测试覆盖 fast predictor 常速/ramp、非交换旋转、圆锥项、ring 回绕、重锚
   不重复积分、gap/sequence/timestamp 错误、未来样本隔离、lane 切换连续及 Euler
   奇异标志，以及 BMI FIFO/capture 启动边界、generation 失效、DMA 异常、三类 bias
   可观测性、selected ICM numeric fault、单 lane 冲击恢复和普通有界内部 ZARU。

验收结果不能扩展为没有测过的声明：不报告绝对动态角度误差、绝对 group delay、
绝对 yaw 或持续线加速度下的绝对 roll/pitch。
