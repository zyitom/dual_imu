# 双 IMU 性能融合架构构思（硬件不变：BMI088 + ICM-45686）

2026-07-20。目标：在不改硬件的前提下，把双异构 IMU 的融合从"lane 选择 + 故障见证"
升级为真正提升姿态性能的架构。本文是**架构演进构思**，与 [FIX_PLAN.md](FIX_PLAN.md)
（bug 修复闭环）互补，不冲突；FIX_PLAN 待烧录三件套先行验证，本文分阶段叠加其上。
现状基线：`dual_imu_estimator.c` 双独立 6 维 MEKF + `imu_selector` 硬切换，
cross-lane 残差只用于故障检测，BMI088 的信息不进入姿态估计。

---

## 0. 诚实的增益预算 —— 钱在哪里

先把物理上限写死，避免在没有收益的方向上花工程量：

| 途径 | 理论收益 | 判定 |
|---|---|---|
| 双 gyro 加权平均降噪 | ARW 改善 3.5%（方差比 13.6:1，w_icm≈0.93） | 有但肉眼不可见，顺手做，不当卖点 |
| 双 gyro 差分通道：在线互标定（失准/刻度/群延迟/bias 差） | disagreement 门 1.6%→~0.3%，高速机动下未检误差上限 8 dps→1.5 dps @500dps | **主要收益 1** |
| 双加计差分：振动整流（VRE）见证 | 消灭持续振动下"姿态慢慢歪"这类单 IMU 原理性盲区，量级可达数度 | **主要收益 2** |
| 双加计杆臂差分：饱和期旋转见证 | 把冲击期"未观测旋转"的盲猜 4000 dps 膨胀换成测量值上界，直接改善 FIX_PLAN §1 冲击恢复收敛速度 | **主要收益 3** |
| 权重连续过渡取代 lane 硬切换 | 消灭切换瞬态和 output_alignment 残余斜坡 | 工程质量收益 |
| BMI bias 持续在线跟踪（不再只靠 ZARU） | BMI 作为热备时 bias 始终正确；ZARU §12 死锁面进一步缩小 | 鲁棒性收益 |

核心洞察：**异构双 IMU 的信息量主要在差分通道（ω_bmi − ω_icm、f_bmi − f_icm），
不在求和通道**。求和降噪是 √2 天花板且本组合只剩 3.5%；差分通道则携带失准、刻度、
延迟、bias 差、VRE、离心项等一整套单 IMU 原理上不可观测的量。设计重心 = 把差分
通道做成一等公民估计器。

---

## 1. 差分通道的可观测性 —— 为什么它是免费的标定台

把 BMI 视为待标定 lane，ICM 为参考。window 级（2.5 ms 预积分窗，天然共窗对齐）
差分测量：

```text
z ≡ Δθ_bmi − Δθ_icm ≈ Δb·dt + M·Δθ_true + τ·(ω_end − ω_start) + n
```

- `Δb`(3)：两 lane bias 之差（raw 域）。
- `M`(3+3)：BMI 相对 ICM 的失准（反对称 3 参）+ 对角刻度差（3 参）。
  BMI088 cross-axis 1.4% 是当前 `gyro_disagreement_rate_fraction=0.016` 松门的元凶。
- `τ`(1)：残余相对群延迟（timestamp-chain review 已识别为假分歧风险）。

三类参数被**不同工况自然分离**，互不污染：

| 工况 | 激励 | 可观测参数 | 不可观测 |
|---|---|---|---|
| 静止 | ω≈0, ω̇≈0 | Δb | M、τ（无激励，冻结） |
| 匀速旋转 | ω 大, ω̇≈0 | M（信号 ∝ ω） | τ |
| 变速旋转 | ω̇ 大 | τ（信号 = τ·Δω） | — |

信噪比核算：τ=100 µs、ω̇=100 rad/s² 时 τ 项 ≈0.57 dps，对上窗均值噪声
（BMI 窗噪 ≈0.28 dps）单窗 SNR≈2，秒级累积即收敛。M 项在 500 dps 时 1% 失准
= 5 dps，远超噪声。**收敛速度：普通手动机动几秒内。**

Gauge 约定（防重复计费）：标定器的 Δb 定义在 raw 域，只用于(a)差分通道自身、
(b)把 BMI 流校准到 ICM 帧；两个 lane MEKF 各自的 bias 状态照旧独立学习，互不回写。
将来 Phase 4 合并单 MEKF 时，校准后共模 bias 归 fused MEKF 唯一 bias 状态。

---

## 2. 目标架构总览

```text
L0  时间戳链 + 双 lane 预积分                     （现有，不动）
L1  差分标定器 cross_lane_calibrator（新）
      输入: 双 lane preintegrated windows
      状态: Δb(3) + 失准(3) + 刻度差(3) + τ(1)，RLS 或 10 维小 KF
      输出: ① 校准后 BMI 流（对齐 ICM 帧）
            ② disagreement 门收紧 0.016 → ~0.003
            ③ 标定收敛标志（下游权重调度依据）
L2  虚拟 gyro lane（新，标定收敛后启用）
      delta_angle_fused = w_icm·Δθ_icm + w_bmi·Δθ_bmi_calibrated
      w ∝ 1/σ²（Allan 实测定权），|ω|>0.8·BMI 量程时 w_bmi 软斜坡→0
L3  双加计阵列见证（新）
      (a) VRE/振动见证 → gravity update R 连续加权
      (b) 饱和期旋转见证 → mark_rotation_unobserved 的测量上界
      (c) gyro 刻度高速见证（离心项核对）
L4  MEKF 布局
      渐进版（推荐）: 保持双 MEKF + selector，L1/L3 先落地
      终局版（可选）: 单 fused MEKF 吃 L2 虚拟 lane，双 lane MEKF 降级为 monitor
```

工况调度器（贯穿各层，与现有 stationary/impact 状态机合并而非另起炉灶）：

| 工况 | 判据（多为现有信号） | L1 行为 | L2 权重 | gravity |
|---|---|---|---|---|
| REST | stationary_confirmed | 只更 Δb | 均可 | 满权重 + ZARU |
| SMOOTH | rate < 门限 | 更 M（弱） | 加权融合 | 软加权 |
| DYNAMIC | rate/ω̇ 大 | 更 M、τ（强激励） | 加权融合 | R 膨胀（连续） |
| VIBRATION | HF 能量 + 加计对残差 | 暂停（防污染） | 加权融合 | VRE 见证降权 |
| SHOCK/SAT | clipping / motion guard | 冻结 | BMI 先退出→纯 ICM→均饱和走 L3(b) | 关闭，走恢复 |

---

## 3. 模块设计

### 3.1 L1 差分标定器（收益最大，重构最小，先做）

- 独立新模块 `App/fusion/cross_lane_calibrator.[ch]`，输入只依赖两个
  `imu_preintegrated_window_t`，纯函数式可 host 测试。
- 递推最小二乘（遗忘因子 λ≈0.999）或 10 维信息滤波，每窗一次，400 Hz 下
  10×10 运算对 H7 可忽略。
- 准入门：双窗 gyro_valid、无 clipping、无 impact/inhibit 区间、无 hard fault。
  饱和窗和冲击窗**绝不**进标定器（冲击期差分被 g-sensitivity 差污染）。
- 参数分工况门控（§2 表）：静止窗只开 Δb 行；|Δθ| 超阈值才开 M 行；|Δω| 超
  阈值才开 τ 行。避免弱激励下参数互相吸收。
- 收敛判据：参数协方差对角线 < 目标（如失准 <0.05°、刻度 <0.1%、τ <20 µs）
  且连续 N 窗残差白化。收敛后输出 `calibration_converged`，并把
  `estimator_selector_covariance` 中 `gyro_disagreement_rate_fraction` 的
  生效值换成标定后残差统计（目标 ~0.003）。
- 安全性：标定器发散/残差恶化 → 回退到出厂 0.016 门 + 撤销校准输出（沿用
  `calibration_revoke_windows` 思路）。参数带硬限幅（失准 <3°、刻度 <3%、
  τ <1 ms），越限即判故障不采用。
- 直接受益者：selector 假分歧消失（高速机动下）、FIX_PLAN §9.1-3 的离线
  cross-axis 标定被在线版覆盖、"动 yaw 时 pitch/roll 乱变"类症状的残余部分。

### 3.2 L2 虚拟 gyro lane（标定收敛后的顺手收益）

- 在 window 级合成，不做样本级重采样：两 lane 已按公共窗预积分，
  `Δθ_fused = w1·Δθ_icm + w2·(校准后 Δθ_bmi)`，权重按 PSD：w_icm≈0.93。
- 权重连续调度取代硬切换：
  - lane 窗无效/可疑 → 该 lane 权重经 3~5 窗斜坡归零（软切换，参考
    6-Axis-AHRS 的软权重哲学），output_alignment 切换瞬态消失；
  - |ω| > 0.8×2000 dps → w_bmi 斜坡归零（BMI 先饱和）；
  - ICM FIFO 坏样本窗（hi-res 模式 quirk）→ w_icm 暂降，BMI 桥接，
    姿态无缝——这是当前架构做不到的（切 lane 必有对齐瞬态）。
- 渐进版实现：不新建第三个 MEKF，把虚拟 Δθ 喂给**当前 selected lane 的
  MEKF**传播路径即可（bias 语义不变：仍减该 lane 自己的 bias，因为校准后
  BMI 流已在 ICM 帧、Δb 已扣，共模部分与 ICM bias 同义）。

### 3.3 L3 双加计阵列（7.1 mm 基线的定量边界）

基线 Δr = [0, 7.1 mm, 0]（Y 轴）。差分模型：

```text
Δf ≡ f_bmi − f_icm = ω̇×Δr + ω×(ω×Δr) + Δb_a + n
离心项（Δr 沿 Y）= d·[ω_x ω_y, −(ω_x²+ω_z²), ω_y ω_z]
```

信号量级 vs 干扰：

| 工况 | 离心/切向信号 | 主要干扰 | 结论 |
|---|---|---|---|
| 500 dps | 0.54 m/s² | 加计对失准 0.2° × 共模 1g ≈ 0.03 m/s² | 可用：gyro 刻度粗核对 |
| 2000 dps（BMI 饱和点） | 8.6 m/s² | 共模 2~3g 时泄漏 ~0.1 m/s² | 可靠：饱和判据交叉验证 |
| 4000 dps（ICM 饱和点） | 34.6 m/s² | — | 可靠 |
| 100 g 平动冲击 | （旋转未知） | 失准泄漏 ~3.4 m/s² | 只能当数量级上界 |

三个用途，按可靠度排序：

1. **(b) 饱和期旋转见证（最有价值）**：gyro clipping 窗内，反解
   `|ω_⊥Y| ≈ sqrt(|Δf_离心|/d)`，作为 `attitude_mekf_mark_rotation_unobserved`
   的 rotation_std 输入——用**测量的**数量级替代盲配
   `unobserved_rotation_rate_std_rad_s = 4000 dps`。冲击若实际只有 300 dps
   旋转，P 膨胀小一个数量级，NIS 门重开后收敛快得多；若真有 3000 dps，膨胀
   如实做大。同时它是 motion guard 的独立触发源：**未打满量程的强振动/冲击**
   （FIX_PLAN §1 死锁的漏网工况）在加计差分上仍有离心/切向签名 → 补上
   "冲击未命中 motion guard 判据 → P 永不膨胀"的根因盲区。
   盲区声明：绕 Y（基线轴）的纯旋转不可见——见证只覆盖 ω_x、ω_z，接受此限制
   （对 bound 用途足够，因为实际冲击旋转轴不会恰好持续对准基线）。
2. **(a) VRE/振动见证**：非饱和但高 HF 能量窗，低通后的两 lane 比力差
   （扣除杆臂预测项）出现持续偏置 = 至少一颗在整流。单 IMU 下 VRE 与真实
   倾斜原理不可分，双异构（不同 die、不同封装谐振）可分。输出连续标量
   `s_vre ≥ 1` 乘进 gravity R，取代当前 `stationary_accel_pair_limit` 类
   硬门在振动区的角色。
3. **(c) gyro 刻度高速在线核对**：持续快旋时用 Δf 离心项反算 |ω|² 与 gyro
   积分比对，作为 L1 刻度参数的独立旁证（激励充分时精度 ~1%）。

### 3.4 gravity aiding 软加权统一（收编现有硬门）

现状已有四套加计降权/关断机制（dynamic_accel_gate 硬门、adaptive scale、
accel_recovery 准入、post_impact trust）。引入统一连续权重：

```text
R_eff = R0 · s_rate(|ω|) · s_alpha(|ω̇|) · s_vre(加计对) · s_norm(|f|−g)
```

各 s 为平滑函数（如 1 + k·max(0,x−x0)²），保留极端区硬关断。好处：
门边界不再抖动（当前 15 rad/s 硬门附近反复开关会造成信息忽有忽无），
每个 s 可独立 host 测试。这是把 6-Axis-AHRS"软权重 + Q 膨胀 + 斜坡回升"
蓝本推广到整个 gravity 路径。

---

## 4. 反模式 —— 明确不做的事

1. **不做未标定加权平均**：0.5° 相对失准在 500 dps 下注入 4.4 dps 假角速度，
   比 3.5% 噪声收益大两个数量级。L2 必须在 L1 收敛标志之后启用。
2. **不把外参并进 MEKF 全状态**（MIMC-VINS 式 12+ 维在线估计）：那套设计的
   激励来自视觉约束；纯 AHRS 下弱激励时段外参状态会漂并与 bias 互吸。
   分离式标定器 + 分工况参数门控更稳、可单测、可回滚。
3. **不指望噪声域收益**，权重固定按 Allan 定标，不做花哨的自适应权重
   （自适应权重在异构 13:1 方差比下就是 0.93 附近抖动，纯增熵）。
4. **不上学习类去噪**（Brossard RA-L 2020 那支）：收益真实但要 GPU 训练、
   逐器件数据、H7 上推理确定性差；列为离线研究方向，不进本架构。
5. **不动 L0 时间戳链**：τ 参数是对残余误差的在线兜底，不是重做群延迟建模。

---

## 5. 实施路线（每阶段独立可验收、可回滚）

> **2026-07-20 实施状态**：Phase 1（`cross_lane_calibrator.[ch]` 模块 +
> estimator 集成 + 收敛后 selector 门收紧 0.016→0.003）与 Phase 2
> （`attitude_mekf_mark_rotation_unobserved_directional` + 盲窗加计见证 +
> 隐藏旋转 streak 触发源）已实现并通过全部 host 测试（新增 12 组）、
> ASan/UBSan、ARM Debug/Release 构建。§3.4 软加权经核查现有代码已具备
> （rate²+ω̇²+pair² 连续缩放 × MEKF 内部模长自适应），未改码。
> Phase 0 的烧录验证与真实数据回放基准仍待硬件侧执行；Phase 4 未开始。
> 实施中发现并修正：失准雅可比符号（m×ω = −[ω]×m）在首版实现中反号，
> 由合成数据收敛测试当场捕获——回放/合成基准的价值即在于此。

**Phase 0 — 地基（先于一切）**
- 烧录 FIX_PLAN 待烧三件套（§1L1 + §9.1-3 + §12），确认既有闭环。
- 建**双 lane 原始数据回放集**：转台匀速/变速、手持机动、电机振动台、
  真实冲击，各 ≥60 s，含 TIM5 时间戳。host 端回放驱动
  `dual_imu_estimator_process_next` 出姿态基准曲线。
  **没有回放基准，后续所有"性能提升"都不可证。**
- 用回放集实测两 lane Allan 方差，替换 config 里的 datasheet 噪声。

**Phase 1 — L1 差分标定器**
- 新模块 + host 测试（合成已知失准/刻度/τ 数据注入，验证收敛值与真值）。
- 回放验收：高速机动段 selector residual_nis 均值下降 ≥5×；
  disagreement 门收紧后转台 1000 dps 匀速无假分歧。

**Phase 2 — L3(b) 饱和旋转见证**
- 接入 motion guard（新触发源）与 mark_rotation_unobserved（测量 std）。
- 回放验收：冲击回放中，恢复到 2° 以内的时间较现状缩短（FIX_PLAN §1 的
  验收标准直接复用）；无冲击误触发（振动台不触发旋转见证误报）。

**Phase 3 — L3(a) + §3.4 软加权**
- 回放验收：振动台持续 5 min，tilt 漂移较现状降低；静止/慢动性能不退化
  （所有既有 host 套件通过）。

**Phase 4（可选终局）— L2 虚拟 lane（+ 单 fused MEKF）**
- 先做渐进版（虚拟 Δθ 喂 selected MEKF）。仅当 Phase 1~3 收益兑现且
  切换瞬态仍是可测痛点时，再评估单 fused MEKF + 双 monitor 重构。
- 回放验收：ICM 单窗失效注入下输出连续（无 output_alignment 瞬态）；
  静止 ARW 改善 ~3%（对上理论值即算通过，不追求更多）。

计算量核算：L1（10 维 RLS）+ L3（每窗十几次乘加）+ 软权重，合计远小于
现有单个 6 维 MEKF 的传播开销；H7 @480 MHz、400 Hz 窗率无压力。

---

## 6. 一句话总结

硬件不变时，双异构 IMU 的性能潜力不在"平均"而在"差分"：把
ω/f 差分通道做成在线标定器 + 三种物理见证（VRE、饱和旋转、刻度），
让 ICM-45686 继续当主传感器，让 BMI088 从"备胎"变成"全职测量校准与
异常见证器"——这是该组合下工程上可兑现的全部增益，也恰好逐条对上
FIX_PLAN 里最痛的三个症结（冲击死锁、假分歧、振动污染）。
