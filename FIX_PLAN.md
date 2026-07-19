# 2026-07-16 全面诊断记录与修改计划

本文汇总 2026-07-16 对当前代码的全部分析结论，作为后续修改的依据。
分析时代码基线为 `eb0d5ce`，全部结论只诊断、未改码。
生产规范仍以 [IMU_FUSION.md](IMU_FUSION.md) 为准；研究索引见
[FUSION_RESEARCH.md](FUSION_RESEARCH.md)。

---

## 1. 【最高优先级】冲击后姿态永久错误 —— NIS 门死锁

### 1.1 机理

1. 冲击让姿态产生真实 tilt 误差。来源：gyro 饱和样本被拒绝积分（丢转角，
   BMI ±2000 dps 先饱和）、g-sensitivity（BMI 上限 0.1 dps/g，100 g 冲击
   瞬时等效 10 dps 假角速度）、高速率下的刻度误差。
2. 冲击结束后加计恢复正常，但 `attitude_mekf.c:1109` 的 NIS 门
   （99% 卡方门 9.21）因协方差 P 仍小、残差大而**持续拒绝**重力更新。
3. **死锁根因**：协方差膨胀 `attitude_mekf_mark_rotation_unobserved`
   （attitude_mekf.c:902）只有两条触发路径——
   - 双 gyro 均不可用（dual_imu_estimator.c:761）；
   - 已确认的冲击期 gyro 分歧（dual_imu_estimator.c:1125）。
   若冲击未命中 motion guard 判据（两 gyro 未在 3 ms 内共同 clipping，
   例如强振动没打满量程），P 永不膨胀、`post_impact_reacquire` 永不触发，
   NIS 门**永久拒绝**，滤波器锁死在错误姿态上，只有进入静止（ZARU）才可能爬回。
4. 加重因素：`attitude_mekf.c:1028` 的 adaptive_scale 把测量方差最多放大
   100 倍（`accel_variance_scale_max`），即使偶尔过门增益也近乎为零。

已有但触发过窄的恢复设施（可直接复用）：
- `estimator_reseed_tilt_preserving_heading`（dual_imu_estimator.c:160）：
  保 yaw 的 tilt 重播种，实现正确。
- `estimator_enter_post_impact_reacquire`（dual_imu_estimator.c:936）。

### 1.2 修复方案（三层，按顺序实施）

**第 1 层：连续 NIS 拒绝 → 递增协方差膨胀（保证有限时间收敛）——已实施 ✅**

> 2026-07-16 已实现并验证：`estimator_handle_accel_recovery`
> （dual_imu_estimator.c），config 新增 `accel_recovery_stuck_windows=100`、
> `accel_recovery_reseed_windows=400`、`accel_recovery_inflation_std_rad=0.6°`。
> 实施中发现**第二种死锁形态**：膨胀使 NIS 过门后，>20° 的修正被
> `max_attitude_correction_rad` 信任域拒绝（REJECTED_CORRECTION），streak
> 若不计入则永久卡死。已把 REJECTED_CORRECTION 同样计为 stuck 证据，
> 形成两层体系：**误差 ≤20° 膨胀路径自愈；>20° 升级到保 yaw reseed +
> post_impact_reacquire**。reseed 保留 heading gauge，不置
> heading_continuity_lost。新增两个主机测试（未检测冲击 15° 膨胀路径 /
> 30° reseed 路径），49 个 estimator 测试、全部 16 组 host 套件、
> ASan/UBSan、ARM Debug/Release 构建全部通过。
>
> **2026-07-16 晚补丁（重大）**：首版实现漏掉了本节写明的准入条件
> "当窗动态门通过、**加计模长接近 g**"——任何 NIS 拒绝都计入 stuck。
> 后果：持续快转（手甩 yaw 300–500 dps，远低于 860 dps 动态门）时加计含
> 离心/切向污染、NIS **正确地**拒绝，却被当作门卡死证据：0.25 s 后膨胀
> 撑开门吸入污染重力，1 s 后把 tilt 直接 reseed 到污染方向——症状与旧
> thrash 链一模一样（烧新固件后"还是一样"的元凶候选）。已补两个互补
> 准入门：`accel_recovery_norm_tolerance_mps2=0.5`（模长离 g ≤5%）+
> `accel_recovery_max_rate_rad_s=1.0`（|ω|≤57 dps；垂直污染 ≤3 m/s² 时
> 模长几乎不变——√(g²+a²)−g≈a²/2g——但方向已偏 17°，速率门才是真正挡住
> 它的那道）。不满足则 hold（不清也不进）。新增测试：200 dps + 3 m/s²
> 垂直污染 400 窗，禁止任何膨胀/reseed、tilt 保持 <1°。全套件回归通过。

- estimator 层为每个 lane 增加连续 NIS 拒绝窗计数（`ATTITUDE_MEKF_ACCEL_REJECTED_NIS`
  返回值已存在，diagnostics 里已有 `accel_nis_reject_count` 可参考）。
- 拒绝计数超阈值（建议 100 窗 = 0.25 s，且当窗动态门本身通过、加计模长
  接近 g）后，每窗调用 `mark_rotation_unobserved` 做小步递增膨胀
  （建议每窗 rotation_std ≈ 0.5–1°，有 covariance_ceiling 天然封顶）。
- P 涨上去后 NIS 自动落回门内，数学上保证重新收敛。
- 更长超时（建议 400 窗 = 1 s 仍未收敛）直接走已有的
  tilt-preserving reseed + `post_impact_reacquire`。

**第 2 层：拓宽冲击检测（覆盖未饱和冲击）**

- 现状只认"双 gyro 3 ms 内共同 clipping"。增加 delta 触发：
  相邻窗（或窗内）`|Δaccel| > 0.5 g` 或 `|Δgyro| > 100 °/s` 即判冲击候选。
- 数据源现成：预积分窗已保留 `E[ωωᵀ]` 二阶矩、窗首/尾 gyro、accel 均值
  （imu_preintegrator.h）。
- 触发后进入与 clipping 相同的 impact 处理（暂停加计修正 + 后续 reacquire 资格）。
- 参数出处：RoboMaster 实战仓库（见 §7），accDelta 0.5 g / gyroDelta 100 °/s /
  恢复期 0.5 s，作为初始调参值。

**第 3 层：软权重恢复（替代硬门语义，可选但推荐）**

- 思想（来自 6-Axis-AHRS 实战 + VQF 哲学）：**永不零增益**。
- NIS 超门时不再直接 return REJECTED，而是用重度放大的 R
  （如 combined_scale 强制取 `accel_variance_scale_max`）继续执行更新；
  冲击确认期把等效权重压到正常值的 ~5%，之后 0.5 s 线性斜坡收回放大系数。
- 与第 1 层二选一亦可：第 1 层改动最小、语义保守；第 3 层收敛更平滑。
  建议先上第 1 层验证，再评估是否引入第 3 层。

**验收**：
- 主机回放测试：注入 20–60° 瞬时姿态误差（模拟未饱和冲击），验证
  MEKF 在 N 秒内重新收敛（第 1 层预期 ≤ 2 s），且静止误触发率为 0。
- 现有三类可观测性回放、gauge、保 yaw failover 测试全部保持通过。
- 实测：手敲/摔板子，确认姿态恢复且 yaw 语义（相对、连续）不被破坏。

---

## 2. 时间戳全链路审查结论（无需修改）

**结论：不存在导致积分角度爆炸的时间戳路径；两颗传感器的方法都是
当前硬件下的正确解法，无严重逻辑错误。**

逐环节证据：
- 预积分器 dt 被窗边界裁剪，单窗角度数学上限 = 量程 × 2.5 ms
  （BMI 5°/ICM 10°）；乱序拒收（imu_preintegrator.c:329）、gap > 5 ms
  置 flag → 窗 invalid。
- fast predictor 样本间 dt > 800/1250 µs 即 blocked
  （fast_attitude_predictor.c:219），horizon 有上限，ZOH 同门。
- ICM 16-bit 回卷 aliasing（延迟 65.5–98 ms 会被 unwrap 假接受）被三重
  挡住：aliased anchor 被 residual 门拒绝 → 20 ms freshness stale reset
  （icm45686.c:942）→ 全批 invalid。32.7–65.5 ms 延迟走 unwrap 半区拒绝。
- TIM5 64-bit 扩展 UIF+半区判断正确，capture 与 update 同 IRQ 无竞态
  （imu_time.c:317）。
- BMI accel tick 递推仅限严格 anchored batch，stale/causal 双 reset
  （bmi088.c:2480）；**已核实 anchor 用的确实是 PA0 硬捕获时刻**，
  变量名 `request_timestamp_us` 有误导（bmi088.c:2524），建议改名。
- ICM hi-res 20-bit 解析与手册逐字段核对一致（bit19 符号扩展、
  byte17-19 A|G 半字节、量纲对 2^19，icm45686.c:210）。

方法最优性：BMI gyro 逐样本硬捕获 = 理论最优；BMI accel sensor-time
递推 = 该芯片最优解法；ICM 差距均为硬件限制（PE4 软捕获抖动、CLKIN
不可达）或收益极小（时钟估计器双一阶 PLL vs 2 状态 KF）。

**唯一待确认项**：dual_imu.c:1957-1960 `dual_imu_handle_exti` 的 BMI
accel/gyro 分支会把**软时间戳**推进与硬捕获相同的队列。需确认 CubeMX
.ioc 中 PA0/PA1 未同时使能 EXTI；若是 bring-up 残留建议删除分支或加断言。

---

## 3. 群延迟（pipeline delay）—— 正确数值与标定方法

### 3.1 ⚠️ 错误数值警告

曾有分析给出 ICM accel 10 ms / gyro 5 ms——**这是公式错误**
（用了 τ=1/f₃dB，正确为 τ≈1/(2π·f₃dB)，差 6.28 倍），手册中也不存在
该数值。**若填入会人为制造 ~4 ms lane 间错位，比全零更糟，切勿采用。**

### 3.2 正确估算（手册核实的滤波结构，±50% 模型估算）

| 流 | 滤波配置 | 群延迟估算 | 中值建议 |
|---|---|---|---|
| BMI gyro  | 230 Hz BW | ~0.7–1.0 ms | 850 µs |
| BMI accel | OSR4, 145 Hz | ~0.9–1.1 ms | 1000 µs |
| ICM gyro  | UI LPF 200 Hz + FIR AAF | ~0.9–1.3 ms | 1100 µs |
| ICM accel | UI LPF 100 Hz + FIR AAF | ~1.7–2.1 ms | 1900 µs |

ICM 信号链：ADC → FIR AAF（固定系数）→ 插值器 → UI LPF（ODR/16）
（DS-000577 §5 Figure 12）；UI LPF 阶数与 AAF tap 数在 AN-000365。

关键定量：两 gyro lane **相对**差仅 ~0.1–0.5 ms（全零宏下相对对齐
其实不差）；lane 内 accel-gyro 差 BMI ~0.25 ms、ICM ~0.8 ms。
高角加速度时 Δω = ω̇·Δτ（ω̇=1e5 dps/s 时 0.3 ms 差 → 30 dps 假分歧），
是冲击期双 lane 假分歧的贡献因素，但非主因。

### 3.3 正确标定路径（优于手册估值）

- host 上对双 gyro 流做**互相关**直接测相对延迟（手动摇板激励即可，
  2000/3200 Hz 采样分辨亚毫秒无压力），自动吸收 AAF/LPF 未公开细节。
- 融合一致性只需要**相对**延迟；绝对延迟只影响输出延迟规格，优先级最低。
- 使用处已就位：`compensate_pipeline_delay`（dual_imu.c:264）与四个
  delay 宏，填实测值即可。
- 备选：把双 lane 分歧门做成随 |ω̇| 自适应加宽（不标定也能压假分歧）。

---

## 4. VQF 接入评估

- VQF 6D 模式匹配本项目约束（无磁、相对 yaw）。
- **推荐**：MEKF 架构不动，移植两个机制——
  (a) 准惯性系加计低通（τ≈3 s）：加计先旋到 gyro 积分姿态的近惯性系再
  低通，冲击/振动变高频量被滤掉，重力方向持续可用，作为连续 NIS 拒绝后
  的降级观测通道（与 §1 第 3 层同哲学）；
  (b) 静止检测 + bias 估计与现有 ZARU 对照。
- 可选：每 lane 并行 VQF shadow（板上第三意见/host replay 对照，计算量极小）。
- **不推荐**整体替换：VQF 无协方差/NIS 输出，selector 与 AMBIGUOUS 语义
  全要重造，也解决不了 clipping 丢转角、绝对 yaw、1v1 软故障归因。

---

## 5. 架构问答记录（避免重复讨论）

- **"核心解算要不要跑 4 kHz"**：不需要。积分已在原生采样率（预积分器 +
  fast predictor 逐样本），无转动信息丢失；400 Hz 修正率是加计信息带宽
  （100–145 Hz）的 3–4 倍，已饱和。延迟地板是传感器滤波群延迟（~1 ms），
  不是算法频率。
- **"双 IMU 又降噪又快"**：降噪天花板 ~3.5%（ICM 0.0038 vs BMI 0.014
  dps/√Hz，逆方差 ICM 权重 93%），是统计学不是架构问题。响应速度的
  真旋钮是 ICM UI LPF 带宽：ODR/16→ODR/4 群延迟 0.8→0.2 ms；姿态是
  角速度积分（天然低通），放宽带宽对姿态 random walk 影响小，代价是
  速率信号抖动和振动整流风险，需实测振动环境决定。
- **SOTA 对比**：时间戳/完整性架构在学术实现之上；扰动/冲击鲁棒性在
  VQF 之下（即 §1 的差距）；绝对精度在逐台标定加载前无法评比。
  修完 §1 后，"完整性架构 + VQF 级扰动抑制 + 全时间戳链"的组合在
  公开文献中无现成对标物。
- **"BMI088+ICM45686 融合是不是没用"**：降噪融合没用（3.5% 天花板），
  完整性组合是正确选型。原则：**同构双 IMU=精度（双 ICM 可得 √2≈29%），
  异构=完整性（不同工艺/失效模式，故障域独立）**，二者不可兼得。
  本板分工：ICM 干全部精度活，BMI 干故障隔离 + 冲击/振动独立见证
  （§1.2 第 2 层 delta 触发用双 lane 证据裁决"真运动 vs 单 lane 故障"）
  + 自检交叉验证。精度提升全在单 lane：逐台标定收益（交叉轴 1.4%→0.3%，
  ~50 倍于融合收益）>> 死锁修复 >> 群延迟 >> 任何融合方案。
  加权平均输出维持不做（IMU_FUSION.md 7.6）。
- **"大 g 冲击场景用哪颗 / 融合目的裁决"**：主传感器 = ICM45686
  （±32g/±4000dps 双项量程覆盖 BMI，噪声低）。BMI 冲击价值不是量程兜底
  （两项都先饱和）而是独立见证：机械共振特性不同 → 伪影不同 → 一致性
  裁决"真运动 vs 传感器说谎"；且 g-sensitivity 有规格上限（0.1 dps/g）
  可对照 ICM（未公布）。融合目的三裁决：冲击=一半（裁决+知错能力，
  本场景价值最大）；响应=否（由 ICM 滤波带宽与 predictor 决定）；
  零漂=否（BMI bias 更差只拖后腿，yaw 漂移是数学边界）。
  真实硬冲击几十~几百 g，±32g 也会饱和——正解是饱和 fail-closed +
  冲击后 recapture（即 §1 修复），不是换芯片；实测用 motion guard
  saturation 计数统计是否常超 ICM 量程，若是则需专用高 g 传感器
  （ADXL372 类）另议。
- **"双 IMU 是不是只有备胎作用"**：性能维度（精度/响应/零漂）≈零贡献
  属实；可靠性维度有五个独立机制，非"死了切换"：
  ① 软故障检测（冲击致 bias 跳变/卡值/伪影——输出看似正常单 IMU 原理上
  无法自检，互差是唯一在线手段，冲击后 ZRO shift 是真实 MEMS 现象）；
  ② 窗口级补洞（已实现：dual_imu_estimator.c:753 本窗 gyro 不可用即用
  另 lane delta angle 补传播，BMI 冲击丢批 20ms 期间 ICM 顶着不断流）；
  ③ 冲击瞬间真伪裁决（异构共振不同，双现=真运动，单现=伪影）；
  ④ 冲击后可信度证书（双 lane 一致→可信，不一致→degraded 标记）；
  ⑤ 可用性乘积（窗口级小故障两芯片不相关）。
  取舍标准：冲击后"姿态错了但系统不知道"的后果严重 → 双 IMU 刚需；
  后果无所谓 → 砍 BMI 做单 ICM 系统是合法简化。

---

## 6. 大波动时输出异常的完整归因排序

1. §1 NIS 门死锁（主因，姿态错误**持续**而非爆炸）。
2. §3 lane 间群延迟失配 → 高角加速度时假分歧 → impact-suspect/
   AMBIGUOUS/checkpoint restore 被误触发 → 输出退化/回退。
3. gyro 饱和丢转角（BMI 先饱和）→ 姿态跳变后接 1 的死锁。
4. ZYX 欧拉角 pitch→±90° 固有奇异：表示层"爆炸"不是积分错误。
   判断异常以四元数连续性为准，`euler_singular`（85°）置位区间的
   roll/yaw 不作证据。

---

## 7. 参考

- [6-Axis-AHRS](https://github.com/liuskywalkerjskd/6-Axis-AHRS)（MIT，
  x-io Fusion/Madgwick 血统 + 东北大学 T-DT 战队 EKF，RoboMaster 2024
  赛季起实战）：冲击恢复三件套（权重降至 5% 不为零、Q 按 1/w 膨胀、
  0.5 s 斜坡回升）与 delta 触发参数的出处。不要学：固定 dt、7-state
  全四元数 EKF、无 yaw gauge 保护；README 数学有错，只吸收思路。
- [VQF](https://doi.org/10.1016/j.inffus.2022.10.014)
- AN-000365 ICM-456xx User Guide（FIR AAF/UI LPF 细节，待获取）
- 111.html 为手写数学推导（MathJax 引用 CDN，离线不渲染，建议本地化）。

---

## 8. 实施顺序

| # | 内容 | 依据 | 改动量 |
|---|---|---|---|
| 1 | ~~连续 NIS 拒绝 → 递增膨胀 → 超时 reseed~~ **完成 2026-07-16** | §1.2 第 1 层 | 小，estimator 层 |
| 2 | delta 冲击触发 | §1.2 第 2 层 | 小，预积分窗数据现成 |
| 3 | 确认/清理 EXTI 双路径 + request_timestamp 改名 | §2 | 微 |
| 4 | 双 gyro 互相关测相对群延迟，填 delay 宏 | §3.3 | 中，host 工具 |
| 5 | （评估后）软权重恢复 / VQF 准惯性系低通 | §1.2 第 3 层 + §4 | 中 |
| 6 | （可选）ICM LPF 带宽 trade 实验 | §5 | 配置 + 振动验证 |

每步完成后跑全部主机测试矩阵 + ASan/UBSan 回归；§1 修复必须新增
"未饱和冲击注入回放"测试并纳入验收（见 §1.2 验收）。

---

## 9. 官方手册二次提取（2026-07-16，ThirdParty/ 两份 PDF）

### 9.1 可直接运用

1. **ICM 时钟源选择（代码缺口）**：DS-000577 §4.14——内部时钟仅
   "RC 振荡器与 gyro MEMS 振荡器自动选优"模式（选项 b）在所有模式下
   保证规格，官方推荐。当前驱动未写入/回读该配置。→ 确认寄存器默认值，
   加入上电 readback 清单。
2. **时钟钳位可收紧**：内部时钟初始容差 ±1.25% + 温度变化项
   （最坏 ~±2.5%）。当前两驱动 clock_scale 钳位 [0.95,1.05] 偏松，
   建议收紧至 ±3%（icm45686.c:618、bmi088.c:295）。
3. **BMI gyro 交叉轴灵敏度 1.4%（ICM ±0.2%）**：未标定时 500 dps 产生
   最多 7 dps 假分量，随转速线性增长，量级超过群延迟失配——是双 lane
   假分歧的另一（可能更大）来源。→ selector 软分歧门加随 |ω| 线性容差项
   （系数 1.6% 有手册背书）；交叉轴/失准标定优先级提前。
4. **内建自检未使用**：BMI 手册给出完整寄存器级流程（ACC_SELF_TEST 0x6D
   正负极性各 >50 ms，静置条件），ICM 为 response 对照 min/max 限值。
   → 上电流式传输前跑自检，失败 lane 直接降级；这是能打破 1v1 软故障
   歧义的真正独立证据，归入 configuration watchdog roadmap。
5. **标定合理性包络**（超出即拒绝加载标定）：
   gyro NL ±0.05%/±0.05%FS；gyro 交叉轴 1.4%/±0.2%；gyro 初始零偏
   ±1/±0.3 dps；accel NL 0.5%FS/±0.01%(±2g)；accel 交叉轴 0.5%/±0.2%；
   ICM accel 零偏 ±10 mg。（顺序 BMI/ICM。）
6. ICM FIFO 硬件无损压缩：有效容量 ×4（阻塞容忍 ~128→512 ms），
   代价为 host 解压 + parser 复杂度。当前 fail-closed 已足够，**不采用**。

### 9.1.5 板级/参数核对结论（2026-07-16 已核，勿重复检查）

- `rotate_accel_pitch_180`：DM-FC01 用户手册"IMU 配置"节明确两颗 IMU 均为
  ROTATION_PITCH_180，硬编码正确，代码已补出处注释（dual_imu.c:289）。
- `BMI088_ACCEL_SENSOR_TIME_TICK_US=39.0625`：手册原文确认（25.6 kHz）。
- gyro noise density 0.014/0.0038 dps/√Hz：与手册一致（estimator 逐 lane 覆盖）。
- `accel_direction_std=1.0°`：数据表白噪声折算仅 ~0.07°，1° 是含振动/扰动
  余量的工程值，方向保守，**不改**。
- 时钟钳位收紧（#3b）：优先用 hi91 实测 ppm 定边界，手册 ±1.25% 作 sanity。
- 杆臂 Z=0：imu_geometry.h 已正确记录原因（封装内 die 位置不可知），无可改。

### 9.2 手册中不存在的信息（勿再检索 PDF）

群延迟数值、ICM g-sensitivity、UI LPF 阶数、FIR AAF tap 数（均在
AN-000365，待获取）、VRE 定量、量程过载恢复时间。

### 9.3 对实施顺序表的增补

| # | 内容 | 依据 | 改动量 |
|---|---|---|---|
| 3a | ICM 时钟源模式确认 + 加入 readback | §9.1-1 | 微 |
| 3b | clock_scale 钳位收紧至 ±3% | §9.1-2 | 微 |
| 2a | selector 分歧门加 1.6%·\|ω\| 容差项 | §9.1-3 | 小，可与 #2 同批 |
| 4a | 上电自检接入 lane 降级 | §9.1-4 | 中 |

---

## 10. 无转台标定路径（2026-07-16 确认）

实施顺序表 #1–#6 全部不需要转台。各标定项的无设备替代方法：

- **相对群延迟**：手持摇板激励 + 双 gyro 流互相关。相对标定不需要真值，
  亚毫秒分辨，直接可用（§3.3）。
- **accel 零偏/刻度/失准**：六面静置或 12+ 姿态椭球拟合，重力即基准；
  精度取决于靠面平整度（平板 + 直角尺）。验收界见 §9.1-5 包络。
- **gyro 刻度**：精确翻转法——板边靠直尺翻精确 180°/360°（几何基准），
  积分角对比，正反双向抵消 bias。可达 ~0.2–0.5%。
- **gyro 失准/交叉轴**：Tedaldi 多姿态法（Tedaldi et al., ICRA 2014；
  开源工具 imu_tk）——静置姿态序列间随手转动，accel 提供姿态参考，
  优化 gyro 参数。可把 BMI 1.4% 交叉轴压到 ~0.2–0.3%。
  与 §9.1-3 的 selector 容差项二选一或叠加。
- **ICM LPF 带宽验证**：实际载体真实振动环境替代振动台。

**无转台做不了的**（与 IMU_FUSION.md 现有门禁一致，保持现状）：
- g-sensitivity K_ga 联合辨识（7.5 节：需 2-DOF 转台，零矩阵保持正确）；
- 温度标定（需热箱，7.4 节门禁不变；环境温度记录仅作穷人版预研）。

注：§3.2 群延迟表为滤波结构模型估算（±50%），非手册原文；手册原文
引用均带文档号（BST-BMI088-DS000-19 Rev1.9、DS-000577 Rev1.0）。

---

## 11. USB 协议层 HI91 状态字重映射 + 零帧修复（2026-07-16 已实施 ✅）

修复两个协议层 bug，供上位机侧对照。改动文件：
`App/protocol/dual_imu_usb_stream.h`、`App/protocol/dual_imu_usb_stream.c`、
`tests/test_dual_imu_usb_stream.c`。仅工作区，未 commit。

### 11.1 MAIN_STATUS 位表对照（官方 HI91 status）

上位机按官方 HI91 status 位表解码，固件此前用自造布局且极性相反
（bit7=ATTITUDE_NOT_CONVERGED、bit11=UTC_UNSYNC 恒 1 等），绿灯语义错乱。
现已对齐官方位表：

| bit | 官方名 | 极性/语义 | 固件实现 |
|-----|--------|-----------|----------|
| 8  | ATT_CONV | 正：1=姿态(tilt)已收敛 | `attitude_valid && attitude_converged && !post_impact_reacquire_active && !attitude_aiding_stale && !rotation_unobserved && !euler_singular`。**不含 heading 连续性**（见 11.5） |
| 9  | GYR_SAT  | 1=陀螺超量程 | 保留真实饱和检测（`gyro_saturation_recent \|\| saturation_history_is_recent(...)`） |
| 10 | ACC_SAT  | 1=加计超量程 | 保留真实加计饱和检测 |
| 12 | WB_CONV  | 正：1=陀螺零偏收敛 | `selected_bias_is_converged(...)` |
| 2  | OD          | 无里程计 | 恒 0 |
| 3  | SOUT_PULSE  | 无同步脉冲 | 恒 0 |
| 4  | UTC_TIME    | 无 UTC | 恒 0（原 UTC_UNSYNC 恒 1 已删除） |
| 5  | MAG_AIDING  | 无磁力计 | 恒 0 |
| 11 | MAG_DIST    | 无磁力计 | 恒 0 |

### 11.2 私有诊断位（HI91 Reserved 位，stock 上位机忽略）

| bit | 私有名 | 语义 |
|-----|--------|------|
| 0  | PRIVATE_GYRO_VALID   | 本帧陀螺样本有限 |
| 1  | PRIVATE_ACCEL_VALID  | 本帧加计样本有限 |
| 6  | PRIVATE_STATIONARY   | 静止已确认（原错置于 bit9，会被上位机误读为 GYR_SAT，已修） |
| 7  | PRIVATE_HEADING_LOST | 航向连续性丢失 |
| 13 | PRIVATE_BMI_FAULT    | BMI088 故障标志非零 |
| 14 | PRIVATE_ICM_FAULT    | ICM45686 故障标志非零 |
| 15 | PRIVATE_STREAM_DROP  | 本会话内发生过丢帧 |

原 `PRIVATE_ATTITUDE_VALID` 与 ATT_CONV 反义重叠，已删除。
`mark_frame_heading_lost()`（航向丢失队列补丁）从"或上 NOT_CONVERGED"
改为"**只置 bit7 PRIVATE_HEADING_LOST**"（不再动 bit8），CRC 重算不变。

### 11.5 ATT_CONV 不含 heading 连续性（关键）

`heading_continuity_lost` 是**粘滞标志**：estimator 里除 init 外只有置 true
的路径，从不清除（真机抓包 48003/48003 帧全程置位）。若把
`!heading_continuity_lost` 折进 ATT_CONV 条件，**绿灯将永远不亮**。
因此 ATT_CONV 只表示 tilt/姿态收敛，与 heading 完全解耦；heading 丢失信息
仅保留在私有 bit7 PRIVATE_HEADING_LOST。**上位机需自行读 bit7 决定是否
重新对零 yaw**，不要用 bit7 去否定 ATT_CONV 绿灯。

### 11.3 禁止全零四元数帧

`fill_frame_data()` 此前 invalid 超过 20 ms 保持窗后仍照发，
euler=0、四元数=(0,0,0,0)——非法四元数，上位机画出跳变到 0 的假曲线。
修复：
- **去掉 20 ms 上限**，改为无限期携带最后一次有效姿态（ATT_CONV 仍为 0，
  纯显示用途），`attitude_held_frame_count` 统计保留；
- **从未有过有效姿态**时，四元数发单位四元数 (w=1,0,0,0)、欧拉角 0，
  而非全零。HI91 四元数为标量在前（`quaternion[0]=w`）。

### 11.4 测试

`tests/test_dual_imu_usb_stream.c` 全部状态位断言已迁移到新位表，
新增/调整覆盖：ATT_CONV 正极性、bit11 不再恒 1、静止时 bit9/bit10 不置位、
无效超窗后四元数非全零、从未有效时为单位四元数、heading-lost 队列补丁只置
bit7 不动 ATT_CONV。验证结果：全部 16 组 host 套件通过（`run_dual_imu_usb_stream_tests.sh`
与 `run_hi91_frame_tests.sh` 重点确认，其余无回归）。

---

## 12. 【新诊断】ZARU 残差门死锁——零偏跑飞后永不恢复（2026-07-16 真机实测）

### 12.1 实测证据（tools/hi91_capture.py，STM32H743 真机）

**剧烈运动会话后（30 s 抓包，48003 帧）：**
- 姿态输出失效 **255 次**（26.2% 帧 invalid），失效起点 |ω| 中位数 340 dps，
  仅 2 次发生在 |ω|<30 dps——失效与高速转动强相关（§6-2 假分歧路径证实）。
- 有效帧间四元数瞬跳 5–20° 共 **1159 次**（相邻帧、远超 2×gyro·dt）——
  恢复/checkpoint 回退时姿态来回甩，即"动 yaw 时 pitch/roll 乱变"的主因。
- 完全静止尾段（std<1 dps）：**输出角速度 x=+5.6、z=−5.3 dps**，
  yaw 漂移 −6.19°/s。输出=原始−学习零偏（dual_imu_estimator.c:2567），
  即 **MEKF 零偏状态错了 ~5.6 dps**（传感器规格 ±0.3~±1 dps 的 5–20 倍）。
- `stationary` 48003 帧无一置位 → ZARU 从未运行。

**JLink 复位后静置重启（60 s 抓包，95956 帧）作对照：**
- 0 次失效、0 次跳变；x/y 零偏 ~0.005 dps；z 零偏 −0.15→−0.11 dps 缓慢收敛；
  yaw 漂移仅 −0.096°/s（−5.8°/min）。
- 结论：**零偏是在剧烈运动中被打飞的（in-run divergence），且系统无法自愈**。
- 次要发现：60 s 理想静置中 `stationary` 仅累计置位 0.9 s（12.1–58.6 s 间
  闪烁），零偏学习被反复打断，WB_CONV/lane calibrated 60 s 未达成——
  静止确认的退出判据过于敏感，需查 reject 计数归因（独立小问题）。

### 12.2 死锁机理

`estimator_stationary_candidate`（dual_imu_estimator.c:1390）的门序：
1. 原始 lane 角速度 |ω|<3°/s（1407 行，**不依赖零偏**，静止时能过）；
2. 加计模长/窗方差/时域方差/双 lane 加计残差（均不依赖零偏，能过）；
3. **残差门（1484–1494 行）**：`|窗口 gyro 均值 − 学习零偏| ≤
   zaru_target_residual_limit = 0.5°/s`。零偏错 5.6 dps → 残差恒 ≈5.6 dps →
   `REJECT_MEAN_RATE` **永久拒绝**。

ZARU 是唯一能修零偏的机制，而它的准入门要求零偏已经基本正确（0.5°/s 内）
——**恢复机制被待修复的错误本身锁死**，与 §1 NIS 门死锁同构。
即使侥幸过门，`zaru_bias_slew_limit = 0.1°/s²` 修 5.6 dps 也需 ~56 s。

零偏为何跑飞：§1/§6 的 NIS 拒绝-恢复 thrash 期间姿态长时间错误，加计残差
经 MEKF 增益持续灌进 bias 状态；饱和丢转角与 lane 切换加剧。修好 §1.2 第 2
层与 §9.1-3 容差门可减少来源，但**必须补上恢复路径**才能闭环。

> **2026-07-16 已实施 ✅**（工作区，与 §1 第 1 层、§9.1-3 容差项同批待烧录）：
> 残差门超限不再立即 reject——在其余全部零偏无关判据通过时进入 **hold 路径**
> （保留 dwell 统计、记 REJECT_MEAN_RATE 归因、连续计数
> `zaru_recovery_reject_streak`），连续 `zaru_recovery_reject_windows=2000`
> （5 s；单 lane 无交叉验证加倍为 10 s）个 admissible 窗后按 lane 用原始
> dwell 均值直接 reseed 零偏（`attitude_mekf_reset` 保姿态、协方差回初始
> 1-sigma），撤销该 lane 标定令 ZARU 重新确认。
> **Admissibility 相对 12.3-1 的设计修正**：原始 dwell 均值按该 lane 重力方向
> **分解**——重力平行（yaw）分量限 `0.5°/s`（保住"1 dps 转台慢转不得学成零偏"
> 的既有测试保证，纯 yaw 慢转不可分是 12.3-3 已接受的残余风险）；重力垂直
> （tilt）分量限 `2.0°/s`（按冲击后 ZRO shift 预算放宽是安全的：真实 tilt
> 慢转会在 5 s dwell 内把重力方向拖过 0.02 弦长稳定门被拒，重力稳定 + 读数
> 恒定只能是零偏）。双 lane 互差门放宽到 `2.0°/s`：单边冲击 ZRO shift 本身
> 就是真零偏、折进 bias 是正确修复，互差门只拦"粗大说谎 lane"，按 0.3°/s
> 收紧反而会对 >0.3 dps 的单边 shift 重现死锁。重力方向不稳直接
> REJECT_GRAVITY_DIRECTION 全量重置（正向运动证据优先于 hold）。
> 新增测试：注入 5.5 dps z 零偏（复现 12.1 实测）→ 限窗内双 lane reseed、
> 静止重确认、零偏 <0.2°/s；1.5°/s tilt 慢转 4000 窗永不 reseed/确认
> （测试恢复窗按物理配比取 800 窗，保证重力门先于 reseed 触发）。
> 验证：49+2 estimator 测试、全部 16 组 host 套件、ASan/UBSan、
> ARM Debug/Release 构建全部通过。

### 12.3 修复方案（与 §1.2 第 1 层同哲学：证据递增 → 升级恢复）

1. 残差门连续拒绝计数：当 `REJECT_MEAN_RATE` 连续 N 窗（建议 2000 窗 = 5 s）
   且其余全部**不依赖零偏**的静止判据持续通过，且**双 lane 原始均值互差**
   在小门内（两颗异构 IMU 同时读到同一近零速率，是"真静止"而非"慢转动"
   的强证据——这正是双 IMU 完整性设计的用武之地），则判定零偏受损，
   进入 bias 恢复模式。
2. 恢复模式：以窗口原始 gyro 均值为目标直接 reseed 零偏（或临时放宽
   slew limit 10 倍斜坡逼近），同时 `mark_rotation_unobserved` 小步膨胀
   yaw 方差以反映零偏重置的不确定性；恢复完成后按正常路径退出。
3. 防误触发界：恢复模式仅在原始 |ω| < stationary_gyro_limit 且加计静止
   判据全通过时可进入；转台慢转场景（真 2°/s 转动）会被双 lane 一致的
   非零均值挡在门外——此时两 lane 均值一致但不为零，与"零偏受损"
   （均值≈0、残差大）可区分？否——慢转时均值即真速率，残差=真速率−零偏
   同样大。区分依据：慢转动下加计重力方向随时间缓慢变化（tilt 分量）+
   时域方差门；纯 yaw 慢转确实不可分，**接受该残余风险**（yaw 零偏本就
   不可观测，reseed 到慢转速率的后果=yaw 漂移，与现状死锁相比不更糟）。
4. 验收：主机回放测试——注入 5 dps 零偏错误 + 静止数据，验证 ≤10 s 内
   零偏恢复、stationary 确认、WB_CONV 置位；真机复现：摇晃后静置桌面，
   yaw 漂移应在 10 s 内从 ~6°/s 收敛回 <0.2°/s。

### 12.4 实施顺序表增补

| # | 内容 | 依据 | 改动量 |
|---|---|---|---|
| 1a | ~~ZARU 残差门死锁修复（12.3）~~ **完成 2026-07-16（未烧录）** | §12 | 小–中，estimator 层 |
| 1b | 静止确认闪烁归因（reject 计数）与退出判据放宽 | §12.1 | 小 |

优先级：1a 与 §8 表 #2（delta 冲击触发）并列最高——#2 减少零偏跑飞的
来源，1a 保证跑飞后能自愈，二者共同闭环冲击鲁棒性。
