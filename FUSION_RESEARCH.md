# BMI088 + ICM45686 融合研究记录

更新时间：2026-07-15。本文保存器件资料、论文入口和候选算法，避免后续重复检索。
它不是当前固件规范；生产行为仍以 [README.md](README.md) 和
[IMU_FUSION.md](IMU_FUSION.md) 为准。

## 1. 当前结论

没有一种仅凭两颗 6 轴 IMU 就能同时做到最高精度、可靠软故障隔离和无漂移航向的
“BMI088 + ICM45686 专用 SOTA”。对当前硬件，最有价值的分工是：

- ICM45686 负责正常工况下的低噪声、高量程主传播。
- BMI088 提供异厂、异构 MEMS 的独立故障域和不同振动传递特性。
- 两条 raw/filter lane 始终独立保留，用于诊断、硬故障隔离和 failover。
- 若要继续挤精度，先增加一个仅作 shadow 的集中式融合输出；它不能取代独立 lane。

当前生产方案“双独立 6-state MEKF + ICM preferred selector”是完整性优先的正确基线。
最值得验证的精度增强候选，是把下面三项组合起来：

1. 每颗 IMU 独立 bias 状态的 Augmented Virtual Filter（AVF）式集中滤波。
2. SOMD/VCE 驱动、逐轴且有上下界的在线噪声协方差。
3. NIS/Huber/CUSUM 完整性门；任一门失败立即退回现有 selector 输出。

这是针对本项目的工程组合，不是某篇论文中已经验证过的单一命名算法。

## 2. 器件证据

| 项目 | BMI088 | ICM45686 | 对融合的含义 |
|---|---:|---:|---|
| gyro noise density（typ.） | 0.014 dps/sqrt(Hz) | 0.0038 dps/sqrt(Hz) | 正常工况应由 ICM 主导 |
| accel noise density（typ.） | X/Y 160、Z 190 ug/sqrt(Hz)，条件见数据表 | 70/80/110 ug/sqrt(Hz)，随量程变化 | 必须按实际量程、带宽重测，不用宣传页固定权重 |
| gyro 初始零偏 | +/-1 dps | +/-0.3 dps | 每颗独立建 bias，不共享一个 bias |
| gyro ZRO 温漂 | +/-0.015 dps/degC | +/-0.005 dps/degC | ICM 纸面更优，仍须逐台热箱标定 |
| 最大 gyro 量程 | +/-2000 dps | +/-4000 dps | BMI clipping 时只能由 ICM 传播 |
| 最大 accel 量程 | +/-24 g | +/-32 g | 强冲击下 ICM 保留测量的概率更高 |
| 最大低噪声 ODR | accel 1.6 kHz、gyro 2 kHz | accel/gyro 6.4 kHz | 当前 1.6/3.2 kHz 已足够；ODR 不等于有效带宽 |
| 时间/FIFO | accel sensor-time；gyro 无逐帧 timestamp | hi-res FIFO 带 timestamp，可用外部 clock | 两芯片仍必须映射到共同 MCU 事件时间 |

BMI088 数据表明确针对无人机/机器人高振动环境，并描述对几百 Hz 以上振动的机械抑制；
其 gyro `g-sensitivity` 给出 `0.1 dps/g`。ICM45686 的 BalancedGyro、低噪声、
`+/-4000 dps`、`+/-32 g` 和 20-bit FIFO 是明显优势，但公开资料没有给出与 BMI 同条件的
`g-sensitivity` 数值。具体电机谐波下谁更好，只能由本板 shaker/rate-table 频响决定。

只按 gyro 白噪声且假设两路独立，逆方差融合的 ICM 权重为：

```text
w_icm = (1 / sigma_icm^2) / (1 / sigma_icm^2 + 1 / sigma_bmi^2)
      ~= 93.1%
```

融合标准差相对单独 ICM 理论上只下降约 `3.49%`；固定 50/50 平均反而约为单独 ICM
的 `1.91x`。所以 BMI 的主要收益是冗余和误差多样性，不是无条件参与平均。

官方资料：

- [Bosch BMI088 data sheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [Bosch BMI088 product page](https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi088)
- [Bosch BMI08x data synchronization](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mis-an006.pdf)
- [TDK ICM45686 data sheet](https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf)
- [TDK ICM45686 product page](https://www.invensense.tdk.com/en-us/products/6-axis/icm-45686)
- [TDK ICM45605/45686 user guide](https://invensense.tdk.com/wp-content/uploads/2024/07/AN-000478_ICM-45605-ICM-45686-User-Guide.pdf)

数据表数字只是先验。最终的 `Q/R` 必须来自当前 PCB、供电、安装、量程、ODR、LPF 和
温度条件下的 Allan deviation、PSD、六面、三轴 rate-table、热箱和 shaker 数据。
两家表中的 typ/max、测试条件和 production-tested/characterized 口径并不完全相同；
表内“初始零偏”也不是 Allan deviation 意义下的 bias instability，不能直接写入随机游走模型。

## 3. 可观测性边界

两颗都旋转、标定并平移到公共刚体坐标后，可写成：

```text
omega_i = omega_true + b_g_i + n_g_i
f_i     = f_origin + alpha x r_i + omega x (omega x r_i) + b_a_i + n_a_i
```

两路差值能观测相对 bias：

```text
d_g = omega_bmi - omega_icm ~= b_g_bmi - b_g_icm + noise
```

但它不能观测两颗共同漂移的 common-mode bias。仅有两颗 6 轴 IMU 时仍然存在：

- 绝对 yaw 不可观，yaw 会长期漂移。
- 绕重力轴的共同 gyro bias 不可由 gravity update 观测。
- 持续线加速度与倾斜不能完全分离。
- 两路软不一致只能被检测；没有第三票或外部观测时，不能可靠判断谁坏。
- clipping 丢失的角增量不能由另一条也 clipping 的数据或事后图优化恢复。

因此，任何声称“双 IMU 自身即可消除 yaw 漂移或完成 1v1 软故障隔离”的方案都不适用。

## 4. 候选算法排名

### 4.1 生产基线：双独立 MEKF + selector

当前实现最适合无外援且完整性优先的场景。它保留每颗 IMU 的 bias、covariance、健康度
和创新历史；硬错误可归因，软分歧输出 `AMBIGUOUS`。缺点是健康时没有利用 BMI 提供的
少量独立白噪声信息，但这个理论损失只有约 3.5%，远小于未标定的相位、温漂和振动风险。

### 4.2 第一优先 shadow：SOMD/VCE + 逐轴 WLS VIMU

Huang 等人的 SOMD 方法针对两路及多路冗余观测在线估计各传感器噪声方差，再用 WLS
生成 virtual IMU。它计算量低，适合先接到现有公共窗后做 shadow：

- 每轴维护 `R_bmi`、`R_icm`，只允许在离线标定上下界内缓慢变化。
- 高角加速度、冲击、clipping、timestamp fault 时冻结在线估计。
- raw lanes 和 selector 保持不变；VIMU 只增加一个候选输出。
- SOMD 主要处理随机噪声，不能把 bias、group delay 或 1v1 故障归因问题一并解决。

参考：[Huang et al., IEEE Sensors Journal 2023](https://doi.org/10.1109/JSEN.2022.3229475)。
更新的动态鲁棒阵列方法继续使用二阶冗余观测、fading memory 和相似度评价，可作为第二个
host 对照：[Liu et al., IEEE Sensors Journal 2026](https://doi.org/10.1109/JSEN.2025.3646454)。

### 4.3 推荐集中模型：AVF 式 9-state attitude filter

2024 年 AVF 把共同 navigation state 与每颗 IMU 的独立 bias 放入同一个滤波器，避免
普通 VIMU 只降白噪声而不估各自 bias。当前仅输出姿态且没有外部平移观测，候选误差状态
应保持为：

```text
delta_x = [delta_theta, delta_bg_bmi, delta_bg_icm]   // 9 states
```

不要在当前观测条件下盲目扩成两个完整 accel bias；没有速度/位置外援时，它们与姿态、
线加速度存在严重耦合。若以后加入 camera/GNSS/encoder，再扩为包含 `v/p` 和每颗
`b_g/b_a` 的 21-state 左右 ESKF。

在每个公共窗内，令两条已去标定先验 bias 的 delta-angle 组成 `y`，共同观测矩阵为
`H = [I; I]`，完整 6x6 测量协方差为 `C`。健康时的 BLUE 输入为：

```text
u_hat = inv(H' inv(C) H) H' inv(C) y
```

`C` 必须包含每条预积分噪声和可确认的交叉协方差；若把两路都受到的板级振动当成独立
白噪声，会得到过度自信的 covariance。可以把加权均值与差值构造成正交观测，或直接按
AVF 推导传播；不能先用同一批数据传播、再把未经协方差处理的差值重复更新一次。

低动态且 accel 门通过时，将两颗 accel 先做杆臂补偿，再以联合 covariance 形成一个
gravity-direction update。强动态时只传播 gyro，不能因为两颗 accel 相互一致就断言它们
测到的是重力。

参考：[Libero and Klein, IEEE TIM 2024](https://doi.org/10.1109/TIM.2024.3370767)。
它的早期预印本标题为 “A Unified Filter for Fusion of Multiple Inertial Measurement
Units”。

### 4.4 自适应鲁棒层：按工况改变 covariance，不直接硬切

每颗、每轴的有效 covariance 可拆成：

```text
R_eff = R_white(T, FS, BW)
      + R_timing
      + R_alignment
      + R_vibration
      + R_unmodelled
```

建议的健康逻辑：

- 瞬态离群：Huber/Student-t 权重或 NIS gate。
- 缓慢漂移：EWMA/CUSUM，避免单窗硬切。
- 饱和、FIFO、timestamp、自检、寄存器回读失败：直接 hard fault。
- 软分歧且无外援：保持当前选中 lane，置 `AMBIGUOUS`，不让自适应权重伪造归因。
- 恢复：连续清洁窗 + 迟滞，权重渐入，不能一步跳回。

GMIS/VCE 为每颗 IMU 保留独立 bias/scale 和 residual，再在线估计 variance component，
很适合异构 BMI+ICM 的建模思想：[Brunson et al., Sensors 2024](https://doi.org/10.3390/s24237754)。
按轴动态组合也有实验证据，但当前更适合做逐轴平滑权重，而不是频繁拼接传感器轴：
[Best Axes Composition](https://doi.org/10.1016/j.robot.2022.104316)。

### 4.5 有外援后：centralized ESKF 或 ACI3 fixed-lag graph

加入 camera、GNSS、encoder、可靠 ZUPT 或载体动力学后，外部 innovation 才能帮助区分
哪颗 IMU 发生软故障，并观测更多 common-mode bias。此时优先集中式 ESKF；需要同时估计
异步时间偏移、IMU 内参/外参且算力充足时，再使用固定窗图优化。

MVIS/ACI3 支持任意数量异步 IMU/gyro，联合 intrinsic、IMU-IMU 时空外参，并用刚体约束
消去不必要的辅助 IMU pose；其可观测性分析也证明增加 IMU 不会消除系统原有 gauge：
[Yang, Geneva and Huang, IJRR 2024](https://doi.org/10.1177/02783649241245726)。

没有外部因子时，把同一批双 IMU 历史改写成 factor graph 只会增加计算量，不会创造
绝对 yaw 或恢复共同 clipping 的转角。

### 4.6 暂不进入控制：学习式融合

R-AFNIO 用自监督重建与 attention 对冗余 IMU 加权，是值得离线对照的学习路线：
[Expert Systems with Applications 2025](https://doi.org/10.1016/j.eswa.2024.125894)。
但目前没有针对 BMI088+ICM45686、跨 PCB/温度/振动的泛化和完整性证据。它只允许读取
日志做 host shadow，不直接控制、不在线写 bias、不取代确定性故障门。

## 5. 针对两颗器件的工况策略

| 工况 | gyro/accel 使用策略 | 完整性状态 |
|---|---|---|
| 静置/低动态 | ICM 约 93% gyro 先验权重；BMI 小权重并监视；两 bias 独立 ZARU | healthy，允许 shadow fusion |
| 普通运动 | ICM 主传播；BMI residual/NIS 持续监视 | healthy 或 suspect |
| 已标定的特定振动频带 | 按本板 PSD/transfer function 逐轴调权 | 权重变化必须有迟滞 |
| 未标定强振动/冲击 | 暂停 accel correction；拒绝 clipping 样本 | degraded |
| 超过 BMI 量程 | ICM-only | degraded，BMI hard fault/saturation |
| ICM FIFO/timestamp/config fault | BMI-only | degraded，ICM isolated |
| 两路软分歧且无外部证据 | 不平均、不反复切换，保持此前 lane | `AMBIGUOUS` |
| 两路共同 clipping | 无有效角增量，增大姿态不确定度 | invalid/heading continuity lost |

不能预设“振动时 BMI 一定更好”。只有 shaker sweep、已知角速度基准和相位标定支持时，
才可构造按频段的 Wiener/BLUE 权重。两路 LPF/group delay 未对齐前做频域拼接，可能把
真实刚体运动变成伪残差，并破坏圆锥积分。

## 6. Shadow 方案的接入门

集中融合输出只有同时满足以下条件才允许参与对比：

1. 两条 lane 覆盖同一公共窗，sample epoch、pipeline delay 和坐标变换有效。
2. 无 saturation、FIFO/sequence/timestamp/config hard fault。
3. delta-angle difference NIS 通过连续窗门限，且没有 `AMBIGUOUS`。
4. 在线 `R` 位于离线 Allan/PSD 给出的资格范围内；估计器在激烈动态时冻结。
5. accel norm、pair residual、angular-rate 和 angular-acceleration 门均通过后，才做重力更新。
6. shadow 的 covariance 通过一致性检查，且 failover 时能无跳变退回 selector。

建议按以下顺序推进：

1. 完成当前待办中的六面、三轴正转、Allan、温箱、group delay、rate-table 和 shaker 数据。
2. 在 host replay 增加 `SOMD-WLS VIMU` 与 `9-state AVF` 两个 shadow 基线。
3. 比较已知真值误差、PSD、NIS/NEES、一致性、clipping 恢复和故障注入，不只看静置 RMS。
4. 在 STM32 上只发布 shadow telemetry，先测 CPU、stack、deadline 和数值稳定性。
5. 只有跨温度、振动、量程和故障数据都证明有净收益，才讨论替换正常区间输出；
   selector/failover 仍保留。

## 7. 论文索引

- [AVF：逐 IMU bias 的统一滤波，IEEE TIM 2024](https://doi.org/10.1109/TIM.2024.3370767)
- [SOMD + WLS VIMU，IEEE Sensors Journal 2023](https://doi.org/10.1109/JSEN.2022.3229475)
- [动态鲁棒多 IMU 融合，IEEE Sensors Journal 2026](https://doi.org/10.1109/JSEN.2025.3646454)
- [GMIS + VCE，Sensors 2024](https://doi.org/10.3390/s24237754)
- [Federated/feedback Federated KF 对照，IEEE Access 2022](https://doi.org/10.1109/ACCESS.2022.3144687)
- [Best Axes Composition，Robotics and Autonomous Systems 2023](https://doi.org/10.1016/j.robot.2022.104316)
- [MVIS/ACI3，IJRR 2024](https://doi.org/10.1177/02783649241245726)
- [多 IMU 阵列可观测性基础，IEEE TSP 2016](https://doi.org/10.1109/TSP.2016.2560136)
- [有独立姿态观测的冗余 gyro FDI，JGCD 2025](https://doi.org/10.2514/1.G008473)
- [R-AFNIO，Expert Systems with Applications 2025](https://doi.org/10.1016/j.eswa.2024.125894)
