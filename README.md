# DM-FC01 双 IMU 融合工程总账

本文是本项目的唯一状态入口，记录 BMI088 + ICM45686 + STM32H743VIT6
双 IMU 系统的需求、硬件事实、已经实现的代码、板上实测、尚未完成的工作和最终
应达到的状态。算法细节和接口说明见 [IMU_FUSION.md](IMU_FUSION.md)。

## 1. 当前结论

目标不是“两颗 IMU 简单平均”，而是异构冗余：每颗 IMU 独立形成一条姿态估计
lane，selector 负责故障检测、隔离和无跳变切换。控制角速度和姿态估计分成快慢
两条路径。

当前代码已经完成双 lane MEKF、公共时间窗预积分、杆臂补偿、加计门控、ZARU、
selector、故障降级、切换连续性、被动温度标定和主机单元测试。当前代码不包含
加热器闭环；PD14/PD15 不应由本模块驱动。

当前还不能称为最终可上控制闭环版本。真实板上已经证明：在 BMI accel 1600 Hz、
BMI gyro 2000 Hz、ICM accel/gyro 1600 Hz 下，主循环逐中断、阻塞式 SPI 寄存器
读取无法持续跟上，会产生事件合并、掉样和 selector 故障。最终方案必须完成 FIFO
批量读取和 DMA；不允许通过把 BMI gyro 降到 1000 Hz、ICM 降到 800 Hz 来隐藏
这个吞吐问题。

状态定义：

- `已完成`：代码已实现，并通过对应 host 测试或板上直接验证。
- `基线`：可以烧录、采数和继续开发，但没有满足最终放行条件。
- `阻塞`：未完成前不得接入飞控/云台闭环。

当前总体状态是 `基线，存在采集通路阻塞项`。

### 1.1 方案收敛记录

早期讨论中的方案不是全部同时成立，最终取舍如下：

- 最初给出的 PB3/PB4/PB5、PA4、PA8 等只是通用示例，已全部让位于
  `DM-FC01_pinout.xlsx` 的实际 PA2/PA3/PA0/PA1、PC13/PE4、SPI1/SPI4。
- 最初考虑用 PA8 MCO 给 ICM CLKIN。该信号没有按此路径连接到当前板的 ICM；
  本版不飞线，BMI 本来也没有同一 CLKIN 能力，因此最终统一采用 sensor-time 到
  TIM5 的仿射回归。下一版 PCB 可把 MCU timer/MCO 正式布到 ICM CLKIN。
- 最初考虑 BMI088 DataSync。当前没有足够网络证据，且 DataSync 会改变 accel
  滤波/带宽选择；本版不启用，也绝不把两个未知中断 pad 同时设为推挽输出。
- 最初考虑一个集中式 9 状态 ESKF 内融合两个陀螺。最终采用每颗 IMU 独立 6 状态
  MEKF lane + selector，因为它能保留独立 innovation、独立 bias 和故障隔离边界。
- “先各跑 Mahony 再平均四元数”只适合早期 bring-up，不是最终架构；Mahony 代码
  已删除。
- ICM 作为默认优先 lane 只是尚无实测时的最后 tie-break，不是永久结论。最终优先
  级由电机全开 PSD、innovation、温度合理性和外部证据决定。
- 恒温对低零飘有高收益，但当前 heater 功率级和热耦合没有完成安全核查，因此本版
  只做被动温标，不实施闭环。

## 2. 需求和不变约束

硬件：

- 主控：STM32H743VIT6。
- IMU 1：BMI088，accel 与 gyro 是两个独立 die 和两个 SPI 从设备。
- IMU 2：ICM45686。
- 驱动来源：`/home/zyi/Downloads/BMI08x_SensorAPI` 和
  `/home/zyi/Downloads/ICM45686-1.1.4`，工程内副本位于 `ThirdParty/`。
- 引脚真相以 `DM-FC01_pinout.xlsx`、供应商手册和板上测量为准，不使用早期讨论中
  的示例引脚。

系统目标：

- 低零飘：被动全温标定、可靠静止零偏更新，未来可选经过硬件验证的恒温；仅靠
  滤波器不能解决不可观 yaw。
- 快响应：控制环使用健康 lane 的高频角速度，姿态估计器不进入角速度内环的关键
  延迟路径。
- 抗振动和冲击：合理片上带宽、量程余量、削顶检测、加计自适应协方差、冲击
  前馈门控、杆臂补偿和 selector 驻留/恢复迟滞。
- 单点故障容错：某颗加计故障不能误伤同芯片健康陀螺；某 lane 陀螺故障后，另一
  lane 可辅助其短时传播；1 对 1 软不一致不能无证据猜坏方。
- 不降级采样规格来迁就软件架构。先解决传输和时间语义，再讨论是否基于实测 PSD
  主动调整 ODR/BW。

## 3. 已确认的引脚与 CubeMX 状态

| 信号 | MCU 引脚 | 当前功能 | 结论 |
|---|---|---|---|
| BMI088 accel CS | PA2 | SPI1 软件 CS | 已验证 |
| BMI088 gyro CS | PA3 | SPI1 软件 CS | 已验证 |
| BMI088 accel DRDY | PA0 | 运行时切为 TIM5_CH1 AF2 | TIM5 未捕获到边沿 |
| BMI088 gyro DRDY | PA1 | 运行时切为 TIM5_CH2 AF2 | 已捕获 |
| ICM45686 CS | PC13 | SPI4 软件 CS | 已验证 |
| ICM45686 数据中断 | PE4 | EXTI4，ISR 快照 TIM5 | 已捕获 |
| ICM45686 另一网络 | PE3 | 当前不参与采集 | 不作为本版 CLKIN |
| BMI088 heater | PD14 | 本版不驱动 | 温控暂停 |
| ICM45686 heater | PD15 | 本版不驱动 | 温控暂停 |
| 红灯 | PE12 | 两颗 IMU 都初始化失败才亮 | 正常时低/灭 |
| 绿灯 | PD11 | 两 lane 完成受控零偏校准才亮 | 无外部静止提示时灭 |
| 蓝灯 | PB15 | 主循环心跳 | 周期变化 |

当前 TIM5 为 1 MHz、32 bit 自由计数，update IRQ 扩展为 64 bit。PA0/PA1 的输入
捕获由固件在初始化时配置，避免重新生成 CubeMX 后丢失。SPI1、SPI4 均为 mode 3，
内核 96 MHz、分频 16，SCK 约 6 MHz。

现有 `.ioc` 只给 UART 和电机定时器分配了 DMA，没有给 SPI1/SPI4 分配 DMA。
最终 FIFO/DMA 通路需要重新打开 CubeMX，建议使用仍空闲的 DMA1 stream：

| 外设请求 | 建议资源 | 方向 | 优先级 |
|---|---|---|---|
| SPI1_RX | DMA1 空闲 stream | peripheral to memory | very high |
| SPI1_TX | DMA1 空闲 stream | memory to peripheral | high |
| SPI4_RX | DMA1 空闲 stream | peripheral to memory | very high |
| SPI4_TX | DMA1 空闲 stream | memory to peripheral | high |

最终分配必须由 CubeMX 根据 DMAMUX 冲突检查确定，不能只照抄 stream 编号。SPI DMA
缓冲不能放 DTCM；应放 D2 SRAM 的专用 section，并二选一：

1. MPU 把 DMA buffer 区设为 non-cacheable；这是优先方案。
2. 每次传输严格做 32-byte 对齐的 D-cache clean/invalidate。

TIM5 capture/update IRQ 优先级应高于 SPI DMA 完成 IRQ。ISR 只打戳、推进事务状态和
入队，不运行解析、标定、滤波或 USB 输出。

## 4. 传感器配置：当前 bring-up 与最终目标

当前寄存器读取代码恢复并保留以下 bring-up 基线；它不是最终 FIFO 配置：

| 通道 | 当前寄存器路径 | 最终 FIFO 路径 | 片上带宽 |
|---|---|---|---:|
| BMI088 accel | 1600 Hz, +/-24 g | 1600 Hz, +/-24 g | OSR4，约 145 Hz |
| BMI088 gyro | 2000 Hz, +/-2000 dps | 2000 Hz, +/-2000 dps | 230 Hz |
| ICM45686 accel | 1600 Hz, 16-bit, +/-32 g | 1600 Hz, 20-bit, 固定 +/-32 g | 约 100 Hz |
| ICM45686 gyro | 1600 Hz, 16-bit, +/-2000 dps | **3200 Hz, 20-bit, 固定 +/-4000 dps** | 约 200 Hz |

ICM 20-byte high-resolution FIFO 会忽略用户 UI FSR：厂商驱动固定返回 accel
`+/-32 g`、gyro `+/-4000 dps`。因此 FIFO 解析必须使用 high-resolution 专用比例，
不能沿用当前 16-bit `+/-2000 dps` 的换算。20 bit 在更大量程下仍比当前 16 bit
路径有更细的角速度量化。

最终“不降级”目标明确为 BMI accel/gyro `1600/2000 Hz`，ICM accel/gyro
`1600/3200 Hz`，而不是刚才为迁就阻塞读取提出的 `1600/1000 + 800/800 Hz`。
最终是否继续调整带宽、notch 和 ICM 3.2 kHz 档位，仍要保留可配置性并用 PSD/
相位数据复核，但吞吐实现必须先按上述最高目标通过。

带宽与 ODR 是不同旋钮。ODR 为窗内积分提供时间分辨率；片上 LPF 是抽取前的
抗混叠措施。云台刚体有效信号通常低于 100--150 Hz，而电机、摩擦轮和结构模态
会在更高频段产生大量能量，因此不把传感器带宽开满。

## 5. 板上已经得到的事实

J-Link：

- 探针 S/N 69655208，目标电压约 3.30 V，SWD 可连接 H743。
- 固件可从 `0x08000000` 下载并 `verifybin` 通过。
- BMI accel ID `0x1E`、BMI gyro ID `0x0F`、ICM WHO_AM_I `0xE9` 均正确。
- TIM5 正常计数，未进入 HardFault；PE12 为低，红灯未亮。
- J-Link 的 `/dev/ttyACM0` 是探针 VCOM，不是目标 USB CDC；当前没有目标 USB
  数据线，因此板上诊断使用 J-Link/GDB 读 RAM 状态。

高 ODR 阻塞读取实测：

- BMI gyro 和 ICM 中断存在，频率接近配置值。
- BMI accel 的 PA0/TIM5_CH1 计数一直为 0，CCR1 不变；PA1/CCR2 正常变化。
- BMI accel 内部 data-ready 状态在变化，INT1/INT2 配置寄存器也能正确回读，说明
  不是 accel 未出数。
- 主循环每个 DRDY 都用阻塞 HAL SPI 读取时，约 5200 次寄存器事务/秒不可持续；
  `coal/drop/latency` 增长，selector 进入 fault/none，400 Hz 融合只能跑到约
  250 Hz。这是软件采集架构失败，不能归咎于 MEKF。

PA0 排查的安全结论：

- `DM-FC01_pinout.xlsx` 把 PA0 标为 BMI088 accel DRDY，但没有展开器件 pad 网络。
- 供应商随板 PX4 目标默认使用 BMI088 accel INT1，因此固件只配置 INT1。
- 曾短暂试验同时启用 accel INT1 和 INT2 推挽，PA0 仍无边沿；该配置已经撤销。
- 不允许再次同时启用两路推挽。若未暴露 pad 与 DataSync 网络相连，可能形成输出
  对驱，存在器件风险。
- 最终仍需示波器/万用表从 BMI accel INT1 pad 到 PA0 做断电连通性与上电波形
  确认。在此之前，最终采集设计不依赖 PA0，而从 BMI accel FIFO sensor-time
  恢复样本时间。

### 5.1 2026-07-14 最终基线烧录快照

烧录文件：`build/RelWithDebInfo/FirstFlightControler.bin`，SHA-256：
`44c298eaf762c0797815381683049b0a34256be72e7ba9e867d397207b07dc19`。
J-Link `loadbin` 和独立 `verifybin` 均成功，目标已 reset/go。

RAM 配置镜像和回读：

- BMI：`1600 Hz/145 Hz/+/-24 g`，`2000 Hz/230 Hz/+/-2000 dps`，verified。
- ICM 当前寄存器路径：`1600 Hz/100 Hz/+/-32 g`，
  `1600 Hz/200 Hz/+/-2000 dps`，FIFO bypass，verified。
- 四个配置 watchdog failure count 均为 0，bus error 均为 0。
- PE12 ODR bit 为 0，红灯灭；PD11 为 0，因为未提供外部静止提示且未完成 ZARU；
  PB15 心跳在采样瞬间为高。

在同一 GDB 会话中明确 `go`，连续运行 `10.011436 s` 后再 halt，避免把调试暂停时间
算进频率，得到：

| 指标 | 计数增量 | 实际频率 |
|---|---:|---:|
| BMI accel IRQ | 0 | 0 Hz |
| BMI accel fallback read | 3844 | 384.0 Hz |
| BMI gyro TIM5 IRQ | 19995 | 1997.2 Hz |
| BMI gyro successful read | 12956 | 1294.1 Hz |
| BMI gyro coalesced | 7038 | 703.0 Hz |
| ICM PE4 IRQ | 16113 | 1609.5 Hz |
| ICM successful read | 11541 | 1152.8 Hz |
| ICM coalesced | 4572 | 456.7 Hz |
| 400 Hz fusion window | 1730 | 172.8 Hz |

同一窗口 BMI gyro capture overrun 增加 1。最终状态为 ICM lane 有有效输出，但
selector 为 `FAULT`；BMI fault `0x120` 表示 accel timing + frozen，ICM fault 为 0。
这不是放行结果，而是高 ODR 真实存在、阻塞读取确实欠吞吐的定量证据。

## 6. 最终采集通路

逐样本阻塞读是临时 bring-up 路径。最终应是三个彼此独立、批量化的事务状态机：

```text
BMI accel FIFO --SPI1--+
                       +--> timestamped stream buffers --> 2.5 ms common windows
BMI gyro FIFO  --SPI1--+

ICM hires FIFO --SPI4------------------------------^ 
```

### 6.1 ICM45686

- 启用 accel + gyro、high-resolution、FIFO timestamp，不启用 compression/ES。
- 最终 accel 1600 Hz、gyro 3200 Hz；accel LPF 使用 DIV16 约 100 Hz，gyro LPF
  也使用 DIV16 约 200 Hz。
- 使用厂商枚举名 `SNAPSHOT` 的 stop-on-full 模式，watermark 先设 2 个 20-byte 包：
  约每 625 us/1600 Hz 触发一次，单批约 40 bytes，仍保留全部 3200 gyro sample/s，
  并平均包含 1 个有效 accel 样本。SPI4 6 MHz 下 40 bytes 纯线时约 53 us。
- 不先用 stream+WM2：受厂商 AN-000364 的 M-1 读取约束，它会留下 1 包并很快再次
  越过 watermark，增加 IRQ/锚点配对复杂度。stop-on-full 在主机严重停顿时显式
  报 FIFO full，随后 hard fault、flush、重建时间轴，比静默覆盖旧样本更安全。
- 在上述 hires + accel/gyro + no compression/ES 配置下，厂商驱动给出的 frame
  size 固定为 20 bytes；解析时仍必须检查 header、INVALID、消息包和 overflow。
  gyro 3.2 kHz、accel 1.6 kHz 时，约每隔一包 accel 字段为 INVALID，这是正常的
  ODR 关系，不能复制上一帧 accel 冒充新样本。
- high-resolution 数据固定按 accel +/-32 g、gyro +/-4000 dps 的 20-bit 比例换算。
- PE4 中断锚定一个 FIFO 批次，DMA 读取 count + payload，解析后逐样本入队。
- timestamp resolution 设为 1 us；watermark 锚定批次末端，必须用逻辑分析/实测
  确认第二包 timestamp 与 PE4 边沿的对应关系，不能仅凭假设写死。
- 将 16-bit sensor timestamp 解卷成 64 bit，先按所选 timestamp resolution 换成
  名义微秒，再递推拟合 `t_mcu = alpha * t_sensor_nominal_us + beta`。ICM 内部时钟
  初差/全温变化可到百分比量级，启动绝对界先用 `[0.95, 1.05]`，异常残差 3 sigma
  剔除；稳定并完成本机温扫后，才按实测斜率范围慢速收紧，不能启动即夹在
  `1 +/- 200 ppm`。
- FIFO overflow、非法 header、时间戳倒退或批内样本数异常必须置 hard fault，复位
  FIFO 后重新建立时间映射；不能把恢复后的第一包和旧时间轴直接相连。

### 6.2 BMI088 gyro

- 维持 2000 Hz/230 Hz，gyro FIFO watermark 先设 1 frame，保持 0.5 ms 控制路径
  新鲜度；DMA 若短时排队，FIFO 仍可一次补读累计帧而不丢样。2.5 ms 估计窗平均
  使用约 5 个样本。
- PA1/TIM5_CH2 捕获每个 DRDY，事件序号和 CCR 时间用于给 FIFO 样本定时并发现
  overcapture/drop。
- SPI1 由统一 arbiter 串行化 accel/gyro CS；gyro 优先，任何时刻只允许一个
  SPI1 DMA transaction。
- BMI gyro FIFO 没有可直接替代 MCU 捕获的完整 sensor-time 时，批次必须与捕获
  队列严格按样本数匹配。无法匹配时整批标 sequence gap，不猜时间。

### 6.3 BMI088 accel

- 维持 1600 Hz/OSR4，使用 accel FIFO；每 2.5 ms 平均约 4 个样本。
- watermark 初值使用 4 frame 对应的约 28-byte 批次，实际长度必须按 Bosch FIFO
  header 与 sensor-time frame 解析结果校验，不能硬编码“永远 28 bytes”。
- 因 PA0 当前无边沿，使用 FIFO sensor-time + MCU 批次读取时刻做仿射时钟回归，
  不以 400 Hz 轮询时刻伪装成原始采样时刻。
- BMI accel sensor-time 为 24 bit、名义分辨率 39.0625 us；先解卷并换成名义时间，
  回归必须允许内部振荡器初差，不能用 MCU 晶振 ppm 直接夹死。
- SPI1 arbiter 在不阻塞 gyro 的前提下周期读取 accel FIFO。
- PA0 修复后可作为 watermark/DRDY 额外锚点，但不能改变 FIFO 内时间戳为主的语义。

### 6.4 事务和故障规则

- DMA 完成后才释放 CS；timeout 时停止对应 DMA、拉高 CS、记录 bus fault，不在 ISR
  中调用传感器完整 re-init。
- 配置 watchdog 必须通过同一个 SPI arbiter 排队，不能在 DMA transaction 中间用
  阻塞寄存器访问抢占总线。
- 原始 DMA buffer、解析后的样本 ring 和估计器消费游标三者分离。
- FIFO watermark 不是样本时间；样本时间只来自 sensor-time 或逐样本硬件 capture。
- 一次漏样可按真实缺失时长放大 Q 后短时传播；超过每流 `max_gap` 的区间必须判
  propagation-invalid，不能跨几十毫秒线性插值。
- 配置 watchdog 分为 BMI accel、BMI gyro、ICM accel、ICM gyro 四个故障域；
  ICM 的中断/FIFO shared 配置同时由两域校验。任一 accel 配置错都不能隔离同芯片
  gyro；gyro ODR/BW/range 回读错误必须进入 gyro selector hard fault。

## 7. 时间戳和公共窗

全局 epoch 是 TIM5 的 64-bit 微秒时间。每颗芯片的 DRDY、FIFO watermark 和传感器
样本时间含义不同，必须显式建模：

```text
t_measurement = affine(sensor_time) - calibrated_pipeline_delay
```

当前四个 pipeline delay 宏均为 0，只是安全占位，不能当成真实值。最终用云台编码器
正弦扫频/互相关分别测：

- BMI accel filter + output delay。
- BMI gyro filter + output delay。
- ICM accel filter + FIFO delay。
- ICM gyro filter + FIFO delay。

两个 lane 在 `[t(k-1), t(k)]` 的同一 2500 us 窗口内积分。边界样本按时间比例
切分，前半窗和后半窗保留真实增量并做圆锥补偿。姿态 timestamp 是窗尾；窗均值
accel/gyro timestamp 是窗中点，二者相差 1250 us 是正确语义。

时间戳质量的杀手级诊断：快速转动时若两 gyro 残差随 `|omega|` 近似线性增大，
斜率对应残余时差。静止噪声看起来正常不能证明时间同步正确。

## 8. 两种距离不能混

### 8.1 两颗 MEMS 之间的杆臂

测的是加计敏感中心到加计敏感中心的三维有向向量，不是到 PCB 中心。供应商 STEP
封装中心：

- BMI088 U11：`(-7.1, 2.401, about 0.462) mm`。
- ICM45686 U13：`(0, 2.401, about 0.450) mm`。

中心距是 **7.1 mm**。两颗器件都经过当前 `ROTATION_PITCH_180` 后，FRD 机体系：

```text
r_ICM_to_BMI  = [0, -0.0071, 0] m
midpoint_to_BMI = [0, -0.00355, 0] m
midpoint_to_ICM = [0, +0.00355, 0] m
```

该杆臂只影响加计一致性；刚体上两点的角速度相同，双 gyro 不做平移补偿。代码使用
窗口的 `E[omega*omega^T]` 和边界角速度计算平均向心/切向项，避免对平均 omega
平方而漏掉零均值角振动的 DC 向心加速度。

### 8.2 IMU 中点到云台机械参考点

还需要从机械 CAD 测 **yaw/pitch 轴线交点指向两 IMU 中点** 的 FRD 三维有向向量。
它通常是厘米级，不是 PCB 中心距，填写：

```c
IMU_REFERENCE_TO_MIDPOINT_X_M
IMU_REFERENCE_TO_MIDPOINT_Y_M
IMU_REFERENCE_TO_MIDPOINT_Z_M
```

测量步骤：在机械总装 CAD 中建立 FRD 坐标系，取 yaw 与 pitch 轴线设计交点为起点，
取 U11/U13 封装中心的中点为终点，直接读取带符号 X/Y/Z。若轴线不严格相交，使用
两轴公垂线的设计参考点，并在标定记录中写明定义。做完整惯导时，“参考点到整机
质心”又是第三条独立向量。

## 9. 最终算法架构

### 9.1 预处理

每个原始样本依次执行：

1. 原始数值、时间戳、sequence、clip/stuck/bus/overflow 标志检查。
2. 每颗 IMU 独立温度偏置多项式、3x3 scale/非正交/残余外参矩阵。
3. 变换到统一 FRD 机体系。
4. 在公共窗内预积分，计算均值、协方差、`E[omega*omega^T]` 和半窗角增量。
5. 把两颗 accel 平移到同一机械参考点，再做 accel 一致性和重力更新。

### 9.2 两条独立 MEKF lane

每颗 IMU 各运行 6 状态 multiplicative error-state EKF：

```text
x_error = [delta_theta(3), delta_b_gyro(3)]
```

- 400 Hz 公共窗传播，二子样圆锥补偿。
- 加计只提供重力方向，使用范数门、自适应 R、NIS 门；冲击期闭锁。
- ZARU 只有“外部静止提示 AND 内部连续静止判据”同时成立才启用。
- 某 lane accel 从上电就坏时，可用另一颗健康 accel 平移后帮助该 lane 初始化；
  gyro 状态和故障域仍保持独立。
- 某 gyro lane 短时故障时可用健康 gyro 辅助传播，并用 PSD 正组合扩大 Q，不能
  伪装为本 lane 自己的健康数据。

选择 MEKF/ESKF 是因为它是姿态惯导的工程标准，误差状态小、四元数归一化自然、
门控和故障协方差接入清晰。UKF 在本问题没有足够收益；IEKF/EqF 对全状态导航和
大初值更有理论优势，但对当前姿态-only、重力-only 场景不是最高收益项。因子图
适合离线/VIO 后端，不进入 H743 的 400 Hz 实时控制链。学习模型可用于离线去噪或
标定研究，不作为本版安全关键估计器。

### 9.3 selector/FDI

- selector 只选择 gyro lane，不选择“整颗 IMU”。accel 来源单独选择。
- 正常时不做 0.5 平均。双 IMU 的核心价值是冗余和诊断，噪声方差改善只是次要收益。
- 双 gyro 在公共窗内用完整 3x3 协方差算 NIS，并包含噪声、时间不确定度、安装
  对准不确定度和辅助传播不确定度。
- 正常时 clip、stuck、bus、FIFO overflow、长 gap、配置回读错误可形成硬故障。
  已知开火/碰撞窗口内，clip 样本仍作废，控制输出仍拒绝该值，但 clip 不写入
  selector 硬锁存；bus、stuck、配置和 timing 故障绝不被冲击门隐藏。
- 单纯双路软不一致时进入 `AMBIGUOUS`，保持当前 lane，不凭两票猜坏方。
- 编码器、视觉或控制器约束通过 `dual_imu_set_isolation_hint()` 提供第三票。非空
  提示 100 ms 自动过期，外部模块必须持续刷新，避免陈旧证据永久隔离健康 lane。
- 驻留时间与恢复迟滞防止来回切换；切换时保持输出四元数连续，再以最大 1 rad/s
  把临时对齐量平滑收敛到新 lane。

### 9.4 快慢双路径

- `dual_imu_get_control_gyro()`：返回 selector 当前健康 lane 的最新高频角速度减本
  lane 估计零偏。最终 FIFO 实现后，每个新 gyro 样本都应更新这条路径。
- `dual_imu_get_state()->quaternion`：400 Hz MEKF 姿态，用于姿态外环和日志。

控制路径仍需针对实际云台做低延迟 2 阶 LPF/notch。固定 notch 频率不能凭经验写死；
先用电机全开 PSD 找结构模态。若电调能提供转速，可在最终验收后升级为转速同步
动态 notch。

## 10. 低零飘的真实边界

只有重力辅助时，roll/pitch 可观，yaw 和沿重力轴的 gyro bias 不可观。没有视觉、
磁航向、里程计或可信云台/底盘航向约束时，任何 MEKF、IEKF、UKF 或神经网络都
不能从数学上消除长期 yaw 漂移。

本版不实现温度闭环，保留以下低风险路径：

- 每颗 accel/gyro 独立三阶被动温度偏置多项式。
- 开机达到热稳定后，控制器确认静止，ZARU 更新 gyro bias。
- 六面 accel 标定、R_ext、多圈编码器 gyro scale、冷到热零偏扫描。
- 工作温度点 2--4 h Allan variance，实测填 MEKF Q/R。

默认弱函数 `app_imu_external_stationary_hint()` 返回 false。因此未接控制器提示时，
`cal=0/0`、绿灯灭是安全预期。不能为了让绿灯亮而取消外部门控，否则恒定慢转会
被学习成零偏。

未来重新评估加热闭环前，必须先确认 PD14/PD15 MOSFET 高低边、负载阻值/功率、
过温失效方式和两颗 IMU 的热耦合。确认前保持 GPIO 安全态。

## 11. 已完成的代码

- `App/imu/bmi088.c`：双 die 初始化、独立 accel/gyro 读、INT1/INT3、配置回读。
- `App/imu/icm45686.c`：初始化、量程/ODR/BW、DRDY、配置回读；当前仍是 FIFO bypass。
- `App/imu/imu_time.*`：TIM5 64-bit 时间与 BMI hardware capture。
- `App/imu/imu_stream_buffer.*`：四条带时间戳/sequence 的 ring buffer。
- `App/imu/imu_calibration.*`：矩阵和被动温度标定、有效性校验、加载钩子。
- `App/imu/imu_geometry.*`：FRD 外参和刚体杆臂补偿。
- `App/fusion/imu_preintegrator.*`：公共窗、真实半窗、圆锥、gap/drop 语义和二阶矩。
- `App/fusion/attitude_mekf.*`：6-state lane、accel/ZARU、Joseph covariance update。
- `App/fusion/imu_selector.*`：NIS、硬故障、suspect/fault/ambiguous、迟滞恢复。
- `App/fusion/dual_imu_estimator.*`：双 lane、cross-aid、selector、无跳变切换。
- `App/imu/dual_imu.*`：采集编排、诊断、外部静止/冲击/隔离接口。
- 旧 Mahony 路径已删除，代码里不存在双 gyro 固定 0.5 平均。

本轮额外修复：

- 撤销临时降频，恢复 BMI gyro 2000 Hz、ICM accel/gyro 1600 Hz。
- BMI accel 只启用 INT1，不再同时驱动 INT1/INT2。
- 配置 watchdog 拆成 BMI accel、BMI gyro、ICM accel、ICM gyro 四个域。
- BMI gyro 配置错误现在会进入 selector hard fault；BMI accel 配置错误不会隔离健康
  BMI gyro。

## 12. 尚未完成的工作，按优先级排序

### P0：发布阻塞

1. 在 CubeMX 为 SPI1/SPI4 配置 TX/RX DMA，建立 D2 SRAM DMA buffer 与 cache 策略。
2. 实现 ICM accel 1.6 kHz/gyro 3.2 kHz high-resolution FIFO、固定 FSR 换算、
   timestamp 和 DMA batch parser。
3. 实现 BMI gyro FIFO、TIM5 capture 对齐和 SPI1 arbiter。
4. 实现 BMI accel FIFO sensor-time 回归，使 PA0 无边沿时仍保留 1600 Hz 原始样本。
5. 上板连续 2 h 证明所有流 `overflow/drop/coalesced=0`，400 Hz 融合无欠速。
6. 断电连通性 + 示波器确认 BMI accel INT1 到 PA0 的真实硬件网络。
7. 增加传感器启动 self-test 状态机；self-test 会扰动传感器配置和数据，完成后必须
   完整重新初始化并重新建立 FIFO/时间映射，不能在控制运行中直接执行。

### P1：时间和标定

1. 用编码器扫频测四路 pipeline delay 和 PE4 EXTI 抖动。
2. 测两 IMU 到云台轴交点的三维杆臂并写入配置。
3. 六面 accel、双 IMU R_ext、多圈 gyro scale、被动温度曲线。
4. 2--4 h Allan variance，替换所有保守 Q/R 默认值。
5. 电机、摩擦轮全开 PSD，确定片上 BW 和软件 notch；不先拍频点。

### P2：系统接入

1. 控制器覆盖 `app_imu_external_stationary_hint()`。
2. 发射/碰撞事件调用 `dual_imu_notify_impact(50000..100000 us)`。
3. 编码器/视觉提供 selector 第三票和可选 yaw 低频量测。
4. 原始双 IMU + 时间戳 + health + innovation 输出到 rosbag/Foxglove。
5. 控制 gyro 增加经过实测整定的轻 LPF/notch；姿态输出可按最新健康 gyro 从窗尾
   外推到控制时刻，但必须同时保留原始窗尾 epoch，不能篡改时间语义。
6. 增加非阻塞恢复状态机：总线/FIFO 故障达到策略阈值后隔离 lane、复位设备、完整
   重配、自检并重新建立时间轴；当前代码只有重试与配置巡检，没有自动 re-init。

### P3：故障注入与性能验收

1. SPI 错误。
2. `+0.5 dps` bias step。
3. `scale x1.02`。
4. 数据冻结。
5. accel/gyro 削顶。

每类记录检测延迟、误报率、输出跳变和恢复时间。双 IMU 1 对 1 系统对缓慢软故障
天然存在对称性，必须用第三票或明确保持 `AMBIGUOUS`，不能伪造“已隔离”。

## 13. 最终放行标准

数据完整性：

- 目标 ODR 下连续 2 h，无 FIFO overflow、DMA error、capture overrun、ring overwrite。
- 每流实际样本率与配置偏差小于 0.5%，持续 drop/coalesced 为 0。
- 400 Hz 公共窗持续运行，无长 gap 被错误插值。
- ICM 时钟回归残差 sigma 小于 20 us；最终 selector 的 timing sigma 使用实测值。

估计性能：

- 静止 60 s，双 lane 公共窗 `delta-theta` 残差 RMS 与理论协方差比在 0.8--1.5。
- 快速转动时残差不随 `|omega|` 线性膨胀。
- 到温后静置 10 min 记录每 lane yaw 漂移；指标必须由整机需求明确，不能只写
  “看起来不漂”。
- 编码器正弦跟踪相位延迟满足控制设计；原先建议目标小于 2 ms，最终以控制相位
  裕度预算为准。
- 冲击/连发时姿态跳变和恢复满足系统需求；原先建议目标 `<0.1 deg`、`<100 ms`，
  未做实弹/等效冲击前不能宣称达到。

鲁棒性：

- 电机全开 2 h 零误切换。
- 五类故障注入均有明确状态转换、检测时延和恢复策略。
- 任一 accel 从启动失效，另一 gyro 随后故障时仍有可解释的 lane/selector 行为。
- selector 切换输出四元数连续，alignment blend 最终收敛。

只有以上全部通过，项目状态才能从 `基线` 改为 `可接控制闭环`。

## 14. 构建、测试和烧录

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

固件链接地址是 `0x08000000`。J-Link 烧录后必须同时执行 `verifybin`，再 reset/go。
烧录成功不等于数据链通过；还必须比较至少两个时间点的 RAM 计数器，计算真实
`irq/read/fusion` 速率，并确认 drop/coalesced/overflow 不再增长。

## 15. 诊断字段和安全解释

- `bmi=1:1E/0F`, `icm=1:E9`：初始化与 ID。
- `irq_hz`：硬件事件频率；FIFO 版应按 watermark 解释，不再等于样本 ODR。
- `read_hz`：寄存器版是事务率；FIFO 版必须拆成 batch rate 与 sample rate。
- `cap=1:a/g`：TIM5 capture 开启与 overcapture。
- `drop/coal/overflow/lat_us`：数据完整性核心指标。
- FIFO/DMA 版还必须分别输出 `fifo_full/parse_error/timestamp_reset/dma_error` 和
  FIFO/ring high-watermark，不能只保留当前摘要计数。
- `fault/wf/nis_milli`：源故障、窗标志和双 gyro NIS。
- `stat=candidate/confirmed/hint`：内部静止、最终静止和外部提示。
- `cfgcal`：是否装载真实标定；0/0 是单位默认值。
- `cfgfault=bmi_a/bmi_g/icm_a/icm_g`：四个独立配置回读故障域。
- `cal=0/0`：没有完成受控 ZARU，不是初始化失败。

红灯 PE12 只表示两颗传感器都初始化失败，不表示“算法所有验收项通过”。绿灯只有
在外部静止提示参与并完成两 lane 校准后才应亮。不要通过修改 LED 条件掩盖缺失的
校准或数据完整性问题。

## 16. 明确不采用的做法

- 不做两个 gyro 固定 0.5 平均。
- 不让 accel 故障直接隔离同芯片 gyro。
- 不跨长时间空洞插值出虚构角增量。
- 不用读取完成时刻冒充样本时刻。
- 不把 FIFO watermark 时刻冒充 FIFO 中每个样本时刻。
- 不同时启用 BMI accel INT1/INT2 推挽来猜板上接线。
- 不在缺少 MOSFET/功率/热耦合确认时开启加热闭环。
- 不用降低 ODR 作为阻塞 SPI 吞吐问题的最终修复。
- 不把 PCB 中心当作杆臂参考点。
- 不宣称重力-only MEKF 可以消除绝对 yaw 漂移。

## 17. 下一次继续开发的起点

下一步不是再换滤波器，也不是继续调整 selector 阈值。正确起点是：

1. 重新生成带 SPI1/SPI4 RX/TX DMA 的 CubeMX 工程。
2. 先独立完成 ICM FIFO + timestamp + DMA，做 2 h 原始流验收。
3. 再完成 SPI1 arbiter、BMI gyro FIFO 与 BMI accel FIFO sensor-time。
4. 数据完整性绿灯后，才开始 Allan、扫频、PSD、温标和故障注入。

任何后续修改都应同步更新本 README 的“当前结论、板上事实、未完成工作和放行标准”，
避免讨论结论与真实代码再次分叉。
