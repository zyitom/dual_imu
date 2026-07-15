# 双 IMU 项目恢复点与待办

更新时间：2026-07-15。本文是中断工作后的第一恢复入口；定稿原理和接口分别见
[README.md](README.md) 与 [IMU_FUSION.md](IMU_FUSION.md)。板级连线只以
`DM-FC01_pinout.xlsx` 为准。

## 1. 当前已经完成

- BMI088：accel `1600 Hz / OSR4 / +/-24 g`，gyro
  `2000 Hz / 230 Hz / +/-2000 dps`；accel FIFO sensor-time、gyro FIFO +
  PA1/TIM5 capture 已进入异步 SPI1 DMA 数据链。
- ICM45686：accel `1600 Hz / ODR/16 / hi-res +/-32 g`，gyro
  `3200 Hz / ODR/16 / hi-res +/-4000 dps`；20-byte FIFO timestamp 已经解卷并
  仿射映射到 TIM5，异步 SPI4 DMA 已接入。
- 双 lane 预积分、400 Hz MEKF、重力切平面更新、selector、保 yaw lane 切换和内部
  有界 ZARU 已实现。
- TIM5_CH3 按固定 `625 us` epoch 发布 quaternion/Euler，输出率 `1600 Hz`；内部仍
  消费选中 lane 的全部 gyro 样本。
- fast predictor 历史只由主线程推进并发布不可变双缓冲 snapshot；TIM5 ISR 读取只读
  snapshot 并做固定复杂度预测，不遍历共享历史。
- gyro 单 lane 连续 saturation 3 ms 才允许硬隔离；双 gyro 在 3 ms 内共同 clipping 或
  任一 accel clipping 视为冲击，20 ms 内抑制 saturation hard latch、100 ms 内暂停
  accel correction。饱和样本本身始终拒绝积分。
- BMI gyro FIFO/capture fault 锁存 20 ms 后可受控恢复：SPI 暂忙 1 ms 重试，真实失败
  退避 1 s；恢复只做 BYPASS、真实 DRDY seed、STREAM 及 gyro 寄存器回读，不做 power
  cycle，首个可信新 batch 前保持该输出无效。
- USB CDC 输出为固定 82-byte HI91 布局；主机置 DTR 后才开始新会话，未打开端口时
  不积压旧帧。invalid frame 最多 20 ms 携带最近可信 payload，但对应 valid bit 必须
  清零；超过 20 ms 才零填。压力和磁场没有硬件来源，固定填 `+0.0f`。
- 高阶温度补偿有独立 production build lock，默认 ARM 构建中显式 enable 也会被拒绝；
  correction matrix 与固定 `c0` 仍可加载。温度有效性、范围、2 s 时效、因果和变化率门
  已实现；qualification 路径另有 accel/gyro 分流历史、输出坐标三维 correction slew、
  10 ms 单步 `dt` 上限和异常往返诊断。production ARM 明确固定 production=1/high-order=0，
  并编译剔除高阶状态与逻辑。当前没有加载逐台温度曲线，也没有启用 `c1..c3`。
- 静态 gyro g 敏感度 `K_ga` 只实现为 qualification 路径：每源使用独立、同源、因果、
  未饱和且有年龄上限的 calibrated accel 证据，在普通 gyro 校准后执行
  `gyro_out = gyro_temp_matrix - K_ga * accel_body`。production ARM 固定
  g-sensitivity=0 并编译剔除历史和修正路径；当前板的矩阵为零且从未启用。单一平放静止
  数据不能辨识该矩阵，仍需重复多姿态和 rate-table/shaker 或 centrifuge 数据。
- 两个 heater enable 在 `HAL_Init()` 后、system clock 配置前先写低并配置为 output，
  `MX_GPIO_Init()` 再次写低。最终 ELF 不含 PWM Start 调用，也没有 heater/温度闭环。
  这不能替代硬件失效安全：MCU 最初复位阶段引脚仍为 Hi-Z，外部门极下拉尚无原理图
  或测量证据。
- 十五组普通 host tests、十四个 C 测试目标的 ASan/UBSan 回归、`App/` 严格 cppcheck，
  以及 Debug、Release、RelWithDebInfo 三个 ARM clean-first build 均通过且无 warning。
  Release BIN 为 `148384 B`，DTCM 占用 `106304 B`，`.imu_dma` 为精确 `2048 B`，最大
  静态栈帧为 `2808 B`；Release BIN SHA-256 为
  `93ec06e7684d3c667a33046debf3baa8c265002631e2e9e5aab4020785afb54d`。

## 2. 当前实板证据

最终刷写前已完整读取并保存目标板 2 MiB Flash：
`/tmp/dual_imu_stm32h743_pre_final_20260715_0817.bin`，大小 `2097152 B`，SHA-256 为
`f1faf1b522eee5a9b22a3a24eb00c92aac5a2c501e2b2aafa497d74f8fbf0435`。当前 Release
已通过 J-Link load，并单独执行 `verifybin`：读取地址 `0x08000000` 的 `148384` bytes，
结果为 `Verify successful`。

此前 `852d7775...` 镜像的冷启动里程碑为：首帧 `518 ms`，`ATT_CONV` 在 `527 ms`
清除，ICM fault 最后出现于 `738 ms`，BMI fault 最后出现于 `2928 ms`，`STATIC` 在
`4928 ms` 置位，`WB_CONV` 在 `6133 ms` 清除。启动期 fault 状态只用于收敛与恢复
时序记录；进入稳定段后没有继续出现。

该旧镜像的 60 s 静置采集收到 `96006` 帧，测得 `1599.928 frame/s`。CRC、resync、fault、
drop、nonfinite、attitude/gyro/accel valid dropout 和三轴同时为零的计数均为 0。首尾各
5 s 均值相比，roll/pitch/yaw 变化约为 `-0.019/-0.004/+0.024 deg`。这只证明本次安装
状态下的短时静置稳定性和数据完整性，不扩展为逐台绝对精度、动态相位或长期漂移指标。

当前用户称实物平放时，两颗加计给出的公共方向彼此一致，但重力在三轴均有大分量。另已
确认厂商资料存在安装旋转冲突：手册和厂商 ArduPilot `hwdef.dat` 为
`ROTATION_PITCH_180`，厂商 PX4 V1.16 `rc.board_sensors` 却对两颗传 `-R 4`，在该版 PX4
中等于 `ROTATION_YAW_180`。当前保留手册/ArduPilot 基线；两种离散旋转都不能单独解释
“平放但三轴大分量”，必须用同一实物六面原始数据和三轴正转裁决，不能用当前单姿态归零。

旧 `/tmp/dual_imu_852d_static_superseded_1156s.csv` 只采集约 1156 s，已经废弃，不得作为
最终验收或与新数据拼接。当前 `93ec...` 的冷启动短测和完整 2 h 静置采集尚待 CDC
重新枚举后从零开始。

## 3. CubeMX 再生成结论与下次检查

当前 `.ioc` 已包含最终引脚、TIM5、SPI DMA、USB CDC、48 MHz USB clock、NVIC 和
cache 基线。HI91 编码、CRC、1600 Hz 软件队列不属于 CubeMX 配置，因此现在不需要在
CubeMX GUI 里再改协议相关项目。2026-07-15 的实际 Generate 已确认这些配置及 CDC
USER CODE 均保留。

本次 Generate 后，CubeMX 根据 `.ioc` 自动生成了 I-cache enable、USB voltage detector
和 SPI1/SPI4 KeepIOState；旧 USER CODE 中对应的重复设置已经删除。当前 I-cache 只
启用一次、D-cache 关闭，USB 电压检测只在 PCD MSP 中启用，KeepIOState 由 `.ioc`
生成，SPI USER CODE 只保留初始化后的 CFG2 寄存器断言。CubeMX 重建的默认
`STM32H743XX_FLASH.ld` 不参与链接。

不要把 CDC endpoint maximum packet size 改为 82；USB FS bulk endpoint 保持 64
byte，82-byte 应用帧由 USB 自动拆包。当前实测 USB IRQ 优先级 0 时 TIM5 miss/drop
为 0、最大发布延迟 19 us，所以暂不把 OTG_FS 改为优先级 5。只有后续负载实测出现
deadline/miss 才重新决定。

每次 Generate Code 后必须逐项检查：

1. `SCB_EnableICache()` 只调用一次，D-cache 仍关闭。
2. TIM5 为 1 MHz、32-bit；CH1/CH2 input capture、CH3 output compare 和对应 IRQ 保留。
3. SPI1 RX/TX 仍为 DMA1 Stream3/4，SPI4 RX/TX 仍为 Stream5/6。
4. `app_init()` 仍位于 `MX_TIM5_Init()` 之后，`app_process()` 仍在主循环。
5. GCC/Clang toolchain 仍引用 `linker/dual_imu_h743.ld`，不能改回 CubeMX 默认 linker。
6. 链接结果中的 `.imu_dma` 仍位于 `0x30040000..0x300407ff`。
7. `usbd_cdc_if.c/.h` 的 DTR 状态、发送缓冲复制和导出函数仍全部位于 USER CODE 区。
8. 重新运行十五组普通 host tests、十四个 C sanitizer 目标、三个 ARM build，再做一次
   短时 CRC/帧率板测。

设备访问只使用稳定路径：CDC 为
`/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_366437643533-if00`，
J-Link serial 为 `/dev/serial/by-id/usb-SEGGER_J-Link_000069655208-if00`，J-Link S/N
为 `69655208`。不要把会随枚举顺序变化的 `/dev/ttyACM*` 编号写入脚本。当前 `93ec...`
Release 已经 load 并独立 `verifybin` 成功；上述冷启动和 60 s 数据属于旧 `852d...`，
新镜像短测及 2 h 采集均尚待执行。

主机采集程序应只打开端口一次，在同一文件描述符上设置 raw/DTR 并 flush input 后
开始按 `5A A5` 找帧；不要先用 `stty` 等另一个进程短暂打开端口，否则操作系统可能把
上一 DTR 会话已经完成的一帧留在 TTY 输入缓冲中。

顶层 CMake 和 linker assertion 会在错误 linker 生效时直接停止构建。CubeMX 生成的
`Core/Inc/Backup`、`Core/Src/Backup` 已加入 `.gitignore`，不作为源码提交。

## 4. 明确待做

1. CDC 重新枚举后先运行 `93ec...` 冷启动短测，再从零采集完整 2 h；同时保存 CSV 和
   采集摘要，核对帧率、CRC、fault/drop、valid、zero-triplet、时间回卷和 Euler 趋势。
2. 由用户实际施加剧烈振动/冲击，同步记录 status、valid、zero-triplet 和恢复时间；
   主机合成测试不能替代该项。
3. 完成逐台 accel、gyro、安装和 temperature 标定并加载校准数据；当前只有本次上电
   的易失 ZARU trim。另用多姿态和受控 rate-table/shaker 或 centrifuge 数据拟合并独立
   验证 `K_ga`，不能使用当前单一平放静置记录冒充标定。
4. 标定四条 sensor pipeline delay；当前宏均为 0，因此不承诺绝对动态相位。
5. 做受控的六面、三轴正转和接近 Euler 奇异区测试，确认 FRD、Hamilton、ZYX 符号。
   同时裁决厂商手册/ArduPilot 的 pitch-180 与厂商 PX4 的 yaw-180 矛盾。
6. 后续保护包括运行期全配置 watchdog、BMI accel/ICM/广义总线恢复、IWDG、掉电安全的
   标定存储，以及 heater enable 外部硬件下拉的确认；不实现 IMU 加热器或温度闭环。
   高阶温补 correction slew 和温度变化率门已实现，但逐台热箱系数、持久化和实物热循环
   验收仍是 production 解锁前置条件。
7. 如需严格识别单帧丢失或保留 64-bit/us 时间，新增版本化扩展协议；固定 HI91 布局
   只有 uint32 millisecond timestamp，没有 sequence。

## 5. 当前不能宣称

- 没有外部航向观测，yaw 只有相对意义并会漂移。
- 持续线加速度与倾斜只靠两颗 6 轴 MEMS 不能完全区分。
- 尚无已知运动基准，不能宣称 gyro 绝对 scale 或完整 group delay 已标定。
- 当前没有 `K_ga` 系数或动态标定数据，不能宣称 gyro g 敏感度已补偿。
- Euler 在 pitch 接近 `+/-90 deg` 时奇异；控制主输出应使用 quaternion。
- 当前 `93ec...` 的完整 2 h 文件尚未开始，完成与分析前不能宣称长期数据完整性通过。
- 软件已尽早将 PD14/PD15 拉低，但复位最初阶段的 Hi-Z 仍需外部硬件下拉证据，不能
  把软件行为表述为已证明 heater 硬件失效安全。
- 工作树包含尚未提交的本项目改动；不要用 reset、checkout 或 CubeMX 输出覆盖
  `App/`、`linker/`、CMake 和已有 USER CODE。
