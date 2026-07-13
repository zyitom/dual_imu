# DM-FC01 BMI088 + ICM45686 双 IMU 实现说明

> 项目状态、硬件实测、未完成事项和最终放行标准以
> [README.md](README.md) 为准。当前高 ODR 配置已经恢复，但逐样本阻塞 SPI
> 通路在板上不能持续跟上；FIFO/DMA 完成前不得接入控制闭环。

## 1. 当前实现结论

当前固件不是两路等权平均，也不再使用旧 Mahony 路径。正式数据链是：

1. 四条独立采样流：BMI088 accel、BMI088 gyro、ICM45686 accel、ICM45686 gyro。
2. 每条流保留 DRDY 时间戳、事件序号、掉样/乱序/长缺口标志。
3. 两颗 IMU 各运行一个 6 状态 MEKF lane，状态为姿态误差和本 lane 陀螺零偏。
4. 400 Hz 公共窗内做带圆锥补偿的角增量预积分；前、后半窗分别精确积分，在窗中点做加计/ZARU 更新。
5. selector 只仲裁陀螺 lane；加计健康和来源单独管理，不会因为一颗加计坏而切走同芯片的健康陀螺。
6. 正常工作时不平均两个陀螺。双 IMU 的主要收益是故障检测、故障隔离和降级备份，而不是把低噪声 ICM 与高噪声 BMI 强行平均。
7. 控制内环通过 `dual_imu_get_control_gyro()` 获取选中健康 lane 的高频物理角速度；MEKF 姿态是另一条较慢路径。

只有重力辅助时，roll/pitch 可观，绝对 yaw 不可观。没有视觉、编码器航向或其他外部航向量测时，任何滤波器都不能消除长期 yaw 漂移。

## 2. 已确认引脚与时钟

| 设备/信号 | MCU | 当前用途 |
|---|---:|---|
| BMI088 accel CS | PA2 | SPI1 软件片选 |
| BMI088 gyro CS | PA3 | SPI1 软件片选 |
| BMI088 accel DRDY | PA0 | TIM5_CH1 输入捕获，AF2 |
| BMI088 gyro DRDY | PA1 | TIM5_CH2 输入捕获，AF2 |
| ICM45686 CS | PC13 | SPI4 软件片选 |
| ICM45686 INT1/DRDY | PE4 | EXTI4，ISR 内快照 TIM5 |
| TIM5 | 1 MHz, 32 bit | 64 bit 软件扩展的全局微秒时基 |

SPI1、SPI4 内核时钟均为 96 MHz，分频 16，实际 SCK 约 6 MHz。它低于 BMI088 的 10 MHz 上限。两颗传感器均使用 SPI mode 3。

PA0/PA1 的 CCR 是硬件锁存时间戳；PE4 仍是 EXTI 软件快照，因此必须用台架数据测量 ICM 的 ISR 抖动。`cap=1:x/y` 表示 BMI 硬件捕获已启用及两通道 overcapture 累计数。

## 3. 传感器配置及原因

| 通道 | ODR | 量程 | 片上带宽 | 目的 |
|---|---:|---:|---:|---|
| BMI accel | 1600 Hz | +/-24 g | OSR4，约 145 Hz | 冲击余量和抗混叠 |
| BMI gyro | 2000 Hz | +/-2000 dps | 230 Hz | 鲁棒备份和控制带宽 |
| ICM accel | 当前 1600 Hz | 当前 +/-32 g | 100 Hz | 冲击余量、抑制 VRE |
| ICM gyro | 当前 1600 Hz | 当前 +/-2000 dps | 200 Hz | 16-bit 寄存器 bring-up |

ODR 高于估计频率是为了保留窗内角增量和圆锥运动；带宽没有拉满，因为云台刚体有效运动通常低于 100-150 Hz，而电机、摩擦轮和结构模态会把更高频段填满。把带宽开满只会增加噪声、振动整流和混叠风险。

ICM 当前明确配置为 FIFO bypass，读取 16-bit UI 寄存器。最终不降级目标是 accel
1600 Hz、gyro 3200 Hz 的 20-byte high-resolution FIFO + 片内时间戳；该模式固定
accel +/-32 g、gyro +/-4000 dps，必须使用专用 20-bit 换算。FIFO/DMA 未完成前，
当前寄存器路径只是 bring-up 基线。

## 4. 到底测哪个距离

### 两颗 IMU 之间

要测的是两个 MEMS 敏感中心之间的三维有向杆臂，不是到 PCB 中心的距离。供应商 STEP 模型中的封装中心为：

- BMI088 U11：`x=-7.1 mm, y=2.401 mm`
- ICM45686 U13：`x=0 mm, y=2.401 mm`

因此封装中心距为 7.1 mm。两颗器件都经过 `ROTATION_PITCH_180` 变换后，在 FRD 机体系内使用：

```text
r_ICM_to_BMI = [0, -0.0071, 0] m
midpoint_to_BMI = [0, -0.00355, 0] m
midpoint_to_ICM = [0, +0.00355, 0] m
```

封装模型不能给出内部 die 的精确 Z 高度，所以当前 Z 差取 0；这一误差相对 7.1 mm 很小，后续可由标定更新。

### IMU 到云台转轴

若要把输出加速度定义在云台机械参考点，还要从机械 CAD 测另一条向量：从 yaw/pitch 轴线交点指向双 IMU 中点，按 FRD 的 X/Y/Z 带符号填写：

```c
IMU_REFERENCE_TO_MIDPOINT_X_M
IMU_REFERENCE_TO_MIDPOINT_Y_M
IMU_REFERENCE_TO_MIDPOINT_Z_M
```

它不是 PCB 几何中心，也不是两个 IMU 的中心距。若做完整惯导，还可能需要从参考点到整机质心的第三条独立杆臂。

### 补偿公式

窗口平均加速度不能用平均角速度直接平方。固件保留 `E[omega*omega^T]`、窗首/窗尾角速度，并对两个加计使用同一套健康陀螺运动统计：

```text
mean(alpha) = (omega_end - omega_start) / T
mean centripetal = (E[omega*omega^T] - tr(E[omega*omega^T]) I) r
f_reference = mean(f_sensor) - mean(alpha) x r - mean centripetal
```

这样零均值高频角振动仍会得到正确的非零向心项，不会在双加计残差中制造直流假故障。

## 5. 时间对齐和预积分

- TIM5 每约 71.6 分钟回卷一次，固件在 update IRQ 中扩展为 64 bit。
- ISR 只记录 CCR/当前计数、事件序号并入无锁环形队列；SPI 寄存器读取在主循环完成。
- 队列合并、overcapture 和读取失败都会表现为序号不连续。
- 小于配置硬间隔的单次 sequence drop 可以带放大协方差传播，但不能参与 FDI 或 ZARU。
- 超过 `max_gyro_gap_us` 的长缺口绝不线性插值成虚构角增量；故障 lane 改用另一健康陀螺辅助传播。
- BMI gyro 最大允许间隔 1250 us；BMI accel、ICM accel/gyro 为 1500 us。
- 两 lane 以共同 2500 us 窗口事务式推进，任何 lane 不会单独跨窗造成比较错位。
- `fused_sample.timestamp_us` 是姿态窗尾 epoch；窗均值 accel/gyro 的时间戳是窗中点，两者相差 1250 us 是有意的正确语义。

四个片上滤波/流水线延迟宏当前默认为 0。必须用编码器正弦扫频测得后填写：

```text
BMI088_ACCEL_PIPELINE_DELAY_US
BMI088_GYRO_PIPELINE_DELAY_US
ICM45686_ACCEL_PIPELINE_DELAY_US
ICM45686_GYRO_PIPELINE_DELAY_US
```

## 6. 两个 MEKF lane 与 selector

每个 lane 独立维护四元数和三轴陀螺零偏。加计只更新重力切平面两个自由度，使用范数门、自适应 R、NIS 门和数值保护。

selector 使用同一公共窗的两路去偏角增量及完整 3x3 协方差计算 NIS：

- 硬故障、饱和、冻结、超时或严格时序故障可立即隔离。
- 两路仅发生软不一致时进入 `AMBIGUOUS`，不会凭两票猜哪颗坏。
- 编码器、视觉或其他独立证据可通过 `dual_imu_set_isolation_hint()` 打破 1v1
  对称性；非空提示 100 ms 自动过期，必须由外部证据持续刷新。
- 加计坏不参与 gyro selector。确认某 lane 加计故障后，其 MEKF 可临时使用另一颗已平移到公共参考点的健康加计；即使加计从上电就坏，健康陀螺 lane 仍可完成降级初始化并作为备份。
- 切换瞬间保持姿态连续，之后以 1 rad/s 的最大校正速率把临时对齐量收敛到新健康 lane；`blend=1` 表示该过程尚未结束。

控制环应使用 `dual_imu_get_control_gyro()`，不要对两个陀螺再做 0.5 平均。该 API 返回选中 lane 的最新原始速率减去本 lane MEKF 零偏，避免 400 Hz 姿态链的额外延迟。

## 7. 静止、零偏和冲击接口

ZARU 必须同时满足：

1. 控制器/编码器给出可信静止提示；
2. 内部双 IMU 角速度、重力范数、加计一致性和 gyro NIS 连续通过约 1 s；
3. 本次 gyro 窗严格有效，无掉样或长缺口。

默认弱函数 `app_imu_external_stationary_hint()` 返回 `false`。这是故意的：仅靠 IMU 无法区分静止和恒定慢转，自动学习会把真实角速度写进零偏。控制器必须覆盖此函数，典型条件是底盘/云台未使能、编码器速度接近零且没有外力动作。

未覆盖时 `cal=0/0`、`hint=0`、绿灯不亮是安全预期，不代表传感器初始化失败。红灯 PE12 仅在两颗传感器都未初始化时点亮；蓝灯 PB15 是主循环心跳。

已知开火/碰撞前调用：

```c
dual_imu_notify_impact(50000U); /* 例如闭锁加计/FDI 50 ms */
```

预期冲击窗口不计入软故障评分；总线、冻结、配置和 timing 硬故障仍保留。

更准确地说：冲击窗内 gyro 削顶样本仍无效且控制 API 会拒绝输出，但该次 clip
不锁存为 lane 故障，避免预期开火触发额外恢复迟滞；总线、冻结、配置和 timing
硬故障始终保留。

## 8. 标定接口，不含加热闭环

本版按要求不实现 PD14/PD15 温度闭环，也不驱动加热器。已实现的是无风险的被动标定层：

```text
calibrated = correction_matrix * (nominal_FRD - bias_polynomial(T))
```

每颗 IMU、accel/gyro 分别有 3x3 scale/非正交/残余外参矩阵，以及三阶温度偏置多项式。默认均为单位矩阵和零偏置，不会假装已有标定数据。

`app_load_imu_calibration()` 是弱加载钩子；参数层可从 FRAM/Flash 取出 `imu_calibration_t`。矩阵必须有限、右手、非奇异，温度多项式在声明工作区间内必须有界，否则拒绝装载。运行中已有样本后禁止热切换标定，以免让 MEKF 状态瞬间失配。

建议标定顺序：六面 accel -> 多姿态残余 `R_ext` -> 编码器匀速多圈 gyro scale -> 冷热扫描被动零偏曲线 -> 工作温度点 2-4 h Allan -> 电机全开 PSD 和扫频时延。MEKF 的 Q/R、selector 的时序和安装不确定度最终都应由这些数据覆盖当前保守默认值。

## 9. USB 诊断

USB CDC 每 250 ms 输出一行。关键字段：

- `bmi=1:1E/0F`, `icm=1:E9`：初始化和芯片 ID。
- `irq_hz`：BMI accel / BMI gyro / ICM DRDY，应接近 1600 / 2000 / 1600 Hz。
- `read_hz`：实际成功读取率；持续显著低于 IRQ 表示主循环/阻塞 SPI 无法跟上。
- `cap=1:a/g`：BMI 硬件捕获开启及 overcapture 数；正常应一直为 0/0。
- `drop`, `coal`, `lat_us`：事件丢失、合并和 IRQ 到读取最大延迟。
- `fault`, `wf`：源级故障和公共窗标志。
- `nis_milli`：双 gyro 残差 NIS x1000。
- `stat=candidate/confirmed/hint`：内部候选、最终静止和外部提示。
- `cfgcal`：是否装载了自定义标定；0/0 表示身份默认值。
- `av`, `gate`, `blend`：融合加计有效、冲击门、切换收敛状态。

如果 `read_hz` 跟不上、`coal/drop` 持续增加、`lat_us` 接近或超过单个 ODR 周期，不得把当前寄存器读取通路用于控制。此时优先迁移 ICM FIFO/片内时间戳，或把 SPI 读取改成事件触发 DMA。

## 10. 验收边界

代码级回归覆盖：队列/序号、100 ms 长缺口、线性 ramp 真半窗、XY 圆锥组合、零均值振动杆臂、MEKF、ZARU 外部门、加计从启动失效、gyro dropout 辅助传播、selector、无跳变切换及收敛、标定矩阵/温补。

上机放行仍必须逐项测量：

1. 静止 60 s：IRQ/read 频率正确，capture overrun=0，持续 drop/coal=0。
2. 手持/编码器扫频：双 gyro 残差不随 `|omega|` 线性膨胀，拟合每路 pipeline delay。
3. 外部静止提示有效后：两 lane ZARU 收敛，10 min yaw 漂移记录达标。
4. 电机全开 2 h：零误切换，PSD 决定是否需要转速同步 notch。
5. 故障注入：SPI 错、0.5 dps bias step、scale x1.02、冻结、削顶，记录检测时延和误报率。
6. 开火/冲击：姿态跳变、恢复时间和硬饱和标志满足系统指标。

当前仍需实测而不能由代码替代的项目是：真实校准系数、Allan Q/R、PE4 EXTI 抖动、四路群延迟、SPI 数据与 DRDY 的关联延迟、电机振动频谱。完成这些之前，固件是可烧录和可采数的鲁棒基线，不应宣称已经达到最终飞控验收。

## 11. 构建与测试

```sh
cmake --build build/Debug --clean-first -j4
cmake --build build/Release --clean-first -j4

./tests/run_imu_calibration_tests.sh
./tests/run_imu_stream_buffer_tests.sh
./tests/run_imu_preintegrator_tests.sh
./tests/run_attitude_mekf_tests.sh
./tests/run_imu_selector_tests.sh
./tests/run_imu_geometry_tests.sh
./tests/run_dual_imu_estimator_tests.sh
```

固件产物位于 `build/Debug/FirstFlightControler.{elf,hex,bin}`，链接地址为 `0x08000000`。
