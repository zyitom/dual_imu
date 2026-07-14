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
- USB CDC 输出已改为固定 82-byte HI91 布局；主机置 DTR 后才开始新会话，未打开端口
  时不会积压旧帧。压力和磁场没有硬件来源，固定填 `+0.0f`。
- 12 组主机测试以及 Debug、Release、RelWithDebInfo ARM 固件均已构建通过。Release
  当前占用约 `140204 B` Flash、`94712 B` DTCM；`.imu_dma` 保留区为精确 2 KiB。
- 2026-07-15 已从当前 `.ioc` 重新 Generate Code；生成后的十二组测试、三个 ARM build
  和 DMA map 均通过。Release SHA-256 为
  `fab6cc8a26862ed23777c7008ba3f8cddefe38563390ac271665f89c925a2051`。

## 2. 当前实板证据

最终 USB 版本的单 DTR 会话短时满负载结果：

- 连续 `10.016 s` 收到 `16027` 个完整帧，测得 `1600.040 frame/s`。
- CRC 错误 0、重同步字节 0、尾部残帧 0、时间戳倒退 0。
- attitude、gyro 和 accel valid 均为 `16027/16027`，gyro/accel 无错误零填；
  pressure/magnetometer 零填 `16027/16027`；协议队列 drop、deadline miss 均为 0。
- TIM5 compare missed 0、tick drop 0、姿态发布 ring drop 0；最大发布延迟 `19 us`。
- FIFO service 峰值 `177 us`，400 Hz estimator 峰值 `671 us`；BMI/ICM FIFO DMA
  error 均为 0。

这证明当前 USB 满负载下的 FIFO、估计器和 1600 Hz 发布调度有余量。它不证明姿态
绝对精度、绝对动态相位、长期稳定性或 2 h 数据完整性。

此前长跑约进行 15 分钟后按用户要求停止，期间板子被实际移动。该段运行没有显示
FIFO/DMA/compare 数据链错误；运动阶段出现的短暂 estimator invalid 不能按静置故障
解释。该记录不能算作 2 h 验收通过，2 h 测试明确延期。

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
8. 重新运行 12 组主机测试、三个 ARM build，再做一次短时 CRC/帧率板测。

本次清单的前七项、十二组测试、三个 ARM build 和 DMA map 已通过。由于目标板 USB
数据口没有枚举（系统当前只有 J-Link `/dev/ttyACM0`），再生成 Release 的短时上板
CRC/帧率测试尚未执行，保留为下面第一项待办。

主机采集程序应只打开端口一次，在同一文件描述符上设置 raw/DTR 并 flush input 后
开始按 `5A A5` 找帧；不要先用 `stty` 等另一个进程短暂打开端口，否则操作系统可能把
上一 DTR 会话已经完成的一帧留在 TTY 输入缓冲中。

顶层 CMake 和 linker assertion 会在错误 linker 生效时直接停止构建。CubeMX 生成的
`Core/Inc/Backup`、`Core/Src/Backup` 已加入 `.gitignore`，不作为源码提交。

## 4. 明确待做

1. 目标板 USB 数据口接回后，刷入当前再生成 Release，重复短时 HI91 CRC、1600 Hz、
   TIM5 miss/drop 和 DMA error 冒烟测试。
2. 用户方便时重新执行完整 2 h 静置数据完整性测试；此前 15 分钟不能复用为通过。
3. 完成逐台 accel、gyro、安装和 temperature 标定并加载校准数据；当前只有本次上电
   的易失 ZARU trim。
4. 标定四条 sensor pipeline delay；当前宏均为 0，因此不承诺绝对动态相位。
5. 做受控的六面、三轴正转和接近 Euler 奇异区测试，确认 FRD、Hamilton、ZYX 符号。
6. 保护阶段后做：clipping 去抖、温度变化率门、运行期配置 watchdog、总线/FIFO
   自动恢复、IWDG 和掉电安全的标定存储。
7. 如需严格识别单帧丢失或保留 64-bit/us 时间，新增版本化扩展协议；固定 HI91 布局
   只有 uint32 millisecond timestamp，没有 sequence。

## 5. 当前不能宣称

- 没有外部航向观测，yaw 只有相对意义并会漂移。
- 持续线加速度与倾斜只靠两颗 6 轴 MEMS 不能完全区分。
- 尚无已知运动基准，不能宣称 gyro 绝对 scale 或完整 group delay 已标定。
- Euler 在 pitch 接近 `+/-90 deg` 时奇异；控制主输出应使用 quaternion。
- 工作树包含尚未提交的本项目改动；不要用 reset、checkout 或 CubeMX 输出覆盖
  `App/`、`linker/`、CMake 和已有 USER CODE。
