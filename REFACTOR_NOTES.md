# 重构接缝分析与手册待挖参数

记录日期:2026-07-16。本文只做记录,不含已实施的改动之外的代码变更。

---

## A. 已完成:ring buffer 死链删除

`accel_buffers` / `gyro_buffers` 是上一版架构的残留。原设计是主循环 `dual_imu_pop_accel()/pop_gyro()`
再喂 preintegrator;现在 `ingest_accel_sample()/ingest_gyro_sample()` 已直接调
`dual_imu_estimator_push_accel/gyro`,ring buffer 就此失去消费者,但 push 还留着。

证据:

- `dual_imu_pop_accel` / `dual_imu_pop_gyro` 全工程零调用、零测试;唯一提及是
  `imu_preintegrator.h` 的一句过时注释。
- `tests/test_imu_stream_buffer.c` 对 accel/gyro buffer 是 0 覆盖(只测 timestamp_queue)。
- 因无人 pop,ring 永远满 → 每次 push 都触发 overwrite → `state.*_buffer_overwrite_count`
  恒等于「总样本数 − 63」,无诊断意义;且这 4 个字段也无人读。

结果:

| | 之前 | 之后 | 变化 |
|---|---|---|---|
| bss | 109592 B | 99272 B | **−10320 B** |
| DTCMRAM | 107.9 KB | 95.5 KB (74.6%) | −11.5% |
| text | 160664 B | 160480 B | −184 B |
| 行数 | — | — | −201 |

测试 16/16 通过,固件链接通过。

**教训:`--gc-sections` 已开(`cmake/gcc-arm-none-eabi.cmake`),删死函数省不了 flash
(仅 −184 B),删死 RAM 才有真收益(−10.3 KB)。后续清理应以 RAM 和热路径为目标,不是行数。**

---

## B. bmi088.c 拆分:接缝在哪(已验证)

现状:2860 行,94 个全局(12 个 volatile)。

### 已否决:按 accel / gyro 切

`bmi088_fifo_kick()` 是 accel die 与 gyro die 抢 SPI1 的**共用仲裁器**,单个函数即触碰
18 个全局,横跨三个域:

| 域 | 全局 | 引用数 |
|---|---|---|
| 共享 | `bmi088_dma_state` | 23 |
| 共享 | `bmi088_dma_tx` | 13 |
| 共享 | `bmi088_active_request_timestamp_us` | 7 |
| 共享 | `bmi088_prefer_gyro`(仲裁公平性) | 5 |
| accel | `accel_request_pending` / `accel_raw_ready` / `accel_batch_ready` | 11 / 9 / 9 |
| gyro | `gyro_request_pending` / `gyro_raw_ready` / `gyro_batch_ready` | 12 / 10 / 10 |
| gyro | `gyro_capture_sync_fault` | 18 |
| gyro | `gyro_recovery_quiesced` | 5 |

按 accel/gyro 切会迫使其中 12~15 个从 `static` 变成跨文件 `extern`:全局数量一个不少,
可见性反而从文件内扩大到整个工程 —— **比现状更糟**。两颗 die 共用一条 SPI 总线,
accel/gyro 不是这份代码的接缝。

### 采纳:按「传输 vs 解析」切

```
bmi088.c 2860行/94全局
        ↓
bmi088_transport.c   ~700行  DMA状态机 + kick仲裁 + submit_* + dma_callback + raw buffers
                             ★ 12 个 volatile 与全部 ISR 耦合关在此文件
bmi088_accel_parse.c ~600行  纯函数: raw bytes + sensortime + clock_sync → batch
                             ★ 零 volatile,可 host 单测;13-bool 问题在此
bmi088_gyro_parse.c  ~500行  纯函数: raw bytes + capture 时间戳 → batch
                             ★ 可 host 单测;4 次重复 epoch 检查在此
bmi088_config.c      ~600行  init + verify_* + soft_reset(开机跑一次,与运行时无关)
bmi088.c             ~400行  门面 + 温度 + health
```

收益:

1. volatile 全部关进 transport.c。当前 12 个 volatile 散在 2860 行里,读解析逻辑时
   全程要担心并发 —— 这是「看不懂」的主因之一。
2. 两个 parse 文件可上 host 单测。**前几轮发现的所有时间戳问题(13 个 bool、
   4 次 epoch 检查、clock sync 映射)全在解析层,而它们现在一行测试都没有**,
   正因为与 DMA/ISR 缠死。切开后即与 `imu_clock_sync` / `imu_selector` 同级:
   可喂假数据跑断言。
3. 跨文件 extern 仅需 3~4 个(transport 交接的 handoff 结构),而非 12~15 个。

建议顺序:**先切 `bmi088_config.c`**(纯位移、零风险、立刻少 600 行日常要读的代码),
跑通测试与固件链接后,再动 transport/parse 那一刀 —— 后者需重新设计 handoff 结构,
风险高,值得单独一轮。

### 附带待办(切分时顺手做)

- `bmi088_process_gyro_fifo` 的 4 次 `bmi088_gyro_capture_epoch_is_current` 调用
  (bmi088.c:2711 / 2730 / 2785 / 2845)收敛为一个 seqlock:开头取 generation、
  提交前验一次。
- `bmi088_process_accel_fifo` 的 13 个有效性 bool 收敛为 `timestamp_provenance`
  三态枚举(TRUSTED_MAPPED / ESTIMATED_BACKDATED / REJECTED),函数体拆成
  parse → classify → emit。**注意:这些 bool 不是防御性编程,每个对应一个真实
  硬件故障模式,删除会导致静默输出错误时间戳。只改表达方式,不减检查。**
- 建议改 `static` 而非删除(只在自身 .c 内使用却暴露在头文件):
  `icm45686_fifo_prepare_count_read`、`icm45686_fifo_parse_dma_response`、
  `icm45686_fifo_dma_transfer_size`、`bmi088_check_accel_configuration`、
  `imu_time_fast_tick_is_running` 等共 15 个。
- 纯遗留可删:`bmi088_read` / `bmi088_read_accel` / `bmi088_read_gyro`
  (bmi088.c:785-918,134 行)、`icm45686_read` —— 已被 FIFO+DMA 路径取代。
  flash 收益为 0(gc-sections),仅为少读 134 行。

### dual_imu.c(2196 行 / 42 全局)

未分析接缝。粗看可分:诊断字段搬运(约 400 行纯 memcpy)、fast attitude 发布/队列、
estimator 桥接。留待 bmi088.c 切完后再评估。

---

## C. 配置档位审计:是不是开最大了

**结论:ODR 上 BMI 顶格、ICM 留了一倍余量;BW 上四路全部选了最保守(最强滤波)档。**

| | accel | gyro |
|---|---|---|
| BMI088 | BW 145 Hz @ ODR 1600 | BW 230 Hz @ ODR 2000 |
| ICM45686 | BW 100 Hz @ ODR 1600 | BW 200 Hz @ ODR 3200 |

### ODR

| 通道 | 当前 | 芯片上限 | 状态 |
|---|---|---|---|
| BMI088 accel | 1600 Hz | 1600 Hz(`BMI08_ACCEL_ODR_1600_HZ` 是 `bmi08_defs.h` 最高档) | **顶格** |
| BMI088 gyro | 2000 Hz | 2000 Hz | **顶格** |
| ICM45686 accel | 1600 Hz | 6400 Hz | 留 4× 余量 |
| ICM45686 gyro | 3200 Hz | 6400 Hz | 留 2× 余量 |

### BW —— 四路都不是最大带宽,而是最强滤波

- BMI088 accel:145 Hz = `BMI08_ACCEL_BW_OSR4`,是 {OSR4=145, OSR2=234, NORMAL=280}
  三档里**最窄**的。
- BMI088 gyro:230 Hz = `BMI08_GYRO_BW_230_ODR_2000_HZ`。ODR 2000 下有两档
  {532, 230},选了**窄**的。
- ICM45686 两路:`LPFBW_DIV_16` = ODR/16,是**分频最多**的档。

与 `README.md:107` 记录的「无频响数据时的保守选择」一致。

### 关键提醒:BW 档位是群延迟失配的根源,且硬件上无法对齐

BMI088 是固定 OSR/IIR 结构,ICM45686 是 ODR/N 可配置 UI filter —— **滤波器阶数和
形状根本不同,调寄存器永远无法让两路的相位/幅度响应对齐。** 因此两路对齐只能在
软件里做(selector 残差比较前套同一个数字 LPF),不能指望调档位解决。

提高 ICM ODR 到 6400 可以减小其自身的采样延迟,但不解决与 BMI 的失配。

---

## D. 手册待挖参数(按重要性)

两份 datasheet 已在仓库内:

- `ThirdParty/BMI08x/bst-bmi088-ds001.pdf`
- `ThirdParty/ICM45686/ds-000577-icm-45686-datasheet.pdf`
- `DM-FC01飞控板用户手册V1.0.pdf`(板级)

### D1. 四条 pipeline group delay ★最高优先级

`dual_imu.c:46-57` 四个宏当前全为 0:

```c
BMI088_ACCEL_PIPELINE_DELAY_US    0U   ← 需查 BMI088 datasheet:OSR4 @ ODR1600 群延迟
BMI088_GYRO_PIPELINE_DELAY_US     0U   ← 需查 BMI088 datasheet:BW230 @ ODR2000 群延迟
ICM45686_ACCEL_PIPELINE_DELAY_US  0U   ← 需查 ICM45686 datasheet:UI filter LPFBW_DIV_16 群延迟
ICM45686_GYRO_PIPELINE_DELAY_US   0U   ← 同上(通常以 ODR 周期数给出)
```

背景:四条通道的时间戳实际都是「数据可用时刻」而非「物理采样时刻」——
BMI gyro 用 DRDY 捕获边沿;BMI accel 用 watermark INT 锚定 sensortime 映射
(`bmi088.c:2522`);ICM 用 FIFO TMST。三者都含各自的滤波群延迟。两路群延迟不同
→ 常值时间偏移 τ → selector 残差 ≈ τ·(dω/dt),**正比于角加速度**,静止时不可见、
晃动时爆炸。与 commit `a108e1a`「still have bug when big fluctuate」吻合。

先填手册标称值即可消掉大部分常值偏移;器件间差异需 shaker/rate-table 标定
(已列在 `PROJECT_STATUS.md:131`)。

### D2. 传感器时钟精度 → clock_sync 的 scale 边界

当前两路都硬编码 ±5%:

```c
// bmi088.c
.minimum_clock_scale = 0.95,  .maximum_clock_scale = 1.05,   // sensortime
// icm45686.c
.minimum_clock_scale = 0.95,  .maximum_clock_scale = 1.05,   // TMST
```

±5% 过宽,`imu_clock_sync_observe()` 的 slope 异常检测形同虚设。应查:

- BMI088 sensortime 时钟精度(通常 ±1~2%)
- ICM45686 TMST RC 振荡器精度 + 温漂

收紧到「手册值 + 温漂余量」能更早抓到时钟异常。注意 `fifo_clock_scale` 诊断字段
(hi91 里的 `ppm=`)可用来验证实测漂移量,先看实测再定边界更稳。

### D3. noise density → MEKF 的 R/Q

`dual_imu_estimator.c:361-362` 当前:

```c
const float bmi_noise_density = 0.014f * degree;   // 0.014 °/s/√Hz
const float icm_noise_density = 0.0038f * degree;  // 0.0038 °/s/√Hz
```

需对手册的 gyro noise density(mdps/√Hz)与 accel noise density(µg/√Hz)复核。
accel 侧是否也已配置需一并检查。

### D4. g-sensitivity 规格

`IMU_CALIBRATION_G_SENSITIVITY_BUILD_ENABLE` 当前 = 0(编译关闭)。启用前需查
BMI088 gyro 的 g-sensitivity 规格(°/s/g)。ICM45686 同。

### D5. 温度系数 → imu_calibration 温补

零偏 / scale 的温度系数,两份手册都有标称值。

### D6. 已填但值得复核

| 常数 | 当前值 | 出处 | 备注 |
|---|---|---|---|
| `BMI088_ACCEL_SENSOR_TIME_TICK_US` | 39.0625 | bmi088.c:43 | 对应 25.6 kHz,复核手册确认 |
| `ICM45686_FIFO_TIMESTAMP_RESOLUTION_US` | 1.0 | icm45686.c | 已配 `TMST_RESOL_1_US`,一致 |
| `BMI088_ACCEL_SCALE_MPS2_PER_LSB` | 24g/32768 | bmi088.c:18 | 按 FSR 推导,正确 |
| `BMI088_GYRO_SCALE_RAD_S_PER_LSB` | 2000dps/32768 | bmi088.c:19 | 按 FSR 推导,正确 |
| `IMU_DM_FC01_CENTER_DISTANCE_M` | 0.0071 (7.1mm) | imu_geometry.h:21 | 来自板级手册/CAD |

### D7. 板级手册(DM-FC01)待确认

- **安装方向**:`dual_imu.c` 的 `rotate_accel_pitch_180()` / `rotate_gyro_pitch_180()`
  硬编码 pitch 180°。应对照板级手册确认两颗 IMU 的实际贴装方向。
- **杆臂 Z 分量**:`imu_geometry.h:18-20` 注明「封装几何无法定位内部 MEMS die 沿 Z」,
  故 Z 初值为 0。若手册/CAD 能给出 die 高度,可填入;否则保持 0 并靠标定替代。
  X/Y 已由 `IMU_DM_FC01_CENTER_DISTANCE_M` 覆盖。

---

## E. 未解的时序问题(上一轮发现,与本文相关)

`fast attitude` 相位预算无余量,且相位随机:

- BMI accel FIFO watermark = 4 帧 @1600 Hz = **2500 µs**,恰等于 preintegrator
  window 2500 µs,且 `estimator_event_watermark()` 取各 stream 最新样本时间的 min
  → watermark 完全被 BMI accel 支配。
- 窗口栅格锚在 `common_epoch_us = init_time_us + window - remainder`(`dual_imu.c:1588`),
  accel INT 栅格锚在传感器自身相位 → 两者相位差 δ **开机即随机落在 [0, 2500) 内**,
  且因 ppm 级时钟失配会缓慢漂移穿过整个区间(100 ppm ≈ 25 秒一轮)。
- `anchor.timestamp_us = output->end_us`,fast tick 每 625 µs 发布,
  `max_prediction_horizon_us = 3000`。一个 anchor 存活 2500 µs,horizon 从
  (δ + 服务延迟) 涨到 (δ + 延迟 + 2500)。要全程不超 3000 需 **δ + 延迟 ≤ 500 µs**。

预测:fast 输出有效率周期性掉坑,且每次开机表现不同。
验证字段(hi91):`fast=总/有效`、`horizon=`、`lat_us=`、`loop=`。

候选修法:BMI accel watermark 降到 2 帧(1250 µs)、或 window 提到 5000 µs、
或把窗口栅格对齐到 accel INT 相位而非 `init_time_us`。
