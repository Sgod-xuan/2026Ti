# WHEELTEC C07A MSPM0G3507 核心板接 NANO 转 UNO 板指南

本文档基于你提供的原理图：

```text
D:\D\WHEELTEC C07A核心板(Ti-MSPM0G3507)附送资料\3.原理图\C07A核心板原理图（MSPM0G3507-48pin）.pdf
```

你的实际连接方式应理解为：

```text
C07A 核心板 H1/H2 排针 -> 飞线 -> NANO 转 UNO 板的 Nano 插座 -> 小车 UNO 接口硬件
```

这里的关键不是 Arduino UNO 侧排针，而是 NANO 转 UNO 板上原本用于插 Arduino Nano 的 30 个孔位。

## C07A 核心板排针定义

从原理图读取到的 C07A 核心板外接排针如下。

### H1 排针

| H1 脚号 | 信号 |
| ---: | --- |
| 1 | 5V |
| 2 | GND |
| 3 | PA25 |
| 4 | PA26 |
| 5 | PA1 |
| 6 | PA0 |
| 7 | PB17 |
| 8 | PB16 |
| 9 | PA12 |
| 10 | PA8 |
| 11 | PA9 |
| 12 | PA27 |
| 13 | PA24 |
| 14 | GND |
| 15 | 3V3 |

### H2 排针

| H2 脚号 | 信号 |
| ---: | --- |
| 1 | 3V3 |
| 2 | GND |
| 3 | PA7 |
| 4 | PB2 |
| 5 | PB3 |
| 6 | PA17 |
| 7 | PA16 |
| 8 | PA14 |
| 9 | PA13 |
| 10 | PB7 |
| 11 | PB6 |
| 12 | PA22 |
| 13 | PA15 |
| 14 | PB20 |
| 15 | PB24 |

## 推荐飞线表

按当前项目的小车逻辑接线，推荐这样把 C07A 接到 NANO 转 UNO 板的 Nano 插座。

| 小车逻辑信号 | NANO 转 UNO 板 Nano 插座 | Nano 脚号 | C07A 接线点 | G3507 信号 | 说明 |
| --- | --- | ---: | --- | --- | --- |
| 巡线 S1 | A0 | 26 | H1-12 | PA27 / A0_0 | 左外侧，黑线 LOW |
| 巡线 S2 | A1 | 25 | H1-4 | PA26 / A0_1 | 左内侧，黑线 LOW |
| 巡线 S3 | A2 | 24 | H1-3 | PA25 / A0_2 | 右内侧，黑线 LOW |
| 巡线 S4 | A3 | 23 | H1-13 | PA24 / A0_3 | 右外侧，黑线 LOW |
| MPU6050 SDA | A4 | 22 | H1-6 | PA0 / I2C0_SDA | I2C 数据 |
| MPU6050 SCL | A5 | 21 | H1-5 | PA1 / I2C0_SCL | I2C 时钟 |
| 左电机 AIN1/IN1 | D9 | 12 | H2-3 | PA7 / TIMA0_C2 | PWM |
| 左电机 AIN2/IN2 | D6 | 9 | H2-4 | PB2 / TIMG6_C0 或 TIMA1_C0 | PWM 或 GPIO |
| 右电机 AIN1/IN1 | D10 | 13 | H2-5 | PB3 / TIMG6_C1 或 TIMA1_C1 | PWM |
| 右电机 AIN2/IN2 | D5 | 8 | H2-6 | PA17 / TIMG7_C0 或 TIMA1_C0 | PWM 或 GPIO |
| 左编码器 B 相 | D2 | 5 | H1-9 | PA12 | 中断输入 |
| 右编码器 B 相 | D3 | 6 | H1-10 | PA8 | 中断输入 |
| 启动按钮 | D4 | 7 | H1-7 | PB17 | 输入，上拉，按下为 LOW |
| 左编码器 A 相 | D7 | 10 | H1-11 | PA9 | GPIO 输入 |
| 右编码器 A 相 | D8 | 11 | H2-9 | PA13 | GPIO 输入 |
| HC-SR04 Trig，可选 | D12 | 15 | H2-12 | PA22 | GPIO 输出 |
| HC-SR04 Echo，可选 | D13 | 16 | H2-13 | PA15 | GPIO 输入，Echo 建议分压 |
| 3.3V | 3V3 | 17 | H1-15 或 H2-1 | 3V3 | 给转接板 3.3V 侧参考电源 |
| 5V，可选 | +5V | 27 | H1-1 | 5V | 只在转接板/外设确实需要 5V 电源轨时连接 |
| 地 | GND | 4 或 29 | H1-2/H1-14/H2-2 | GND | 必须与小车硬件共地 |

## Nano 插座脚号方向

NANO 转 UNO 板的 30 针 Nano 插座通常按 Arduino Nano 脚位排列：

```text
D1/TX  (1)   (30) VIN
D0/RX  (2)   (29) GND
RESET  (3)   (28) RESET
GND    (4)   (27) +5V
D2     (5)   (26) A0
D3     (6)   (25) A1
D4     (7)   (24) A2
D5     (8)   (23) A3
D6     (9)   (22) A4
D7    (10)   (21) A5
D8    (11)   (20) A6
D9    (12)   (19) A7
D10   (13)   (18) AREF
D11   (14)   (17) 3V3
D12   (15)   (16) D13
```

接线时要看 NANO 转 UNO 板的 Nano 丝印、缺口方向或原理图，不要只凭左右猜。

## 关于 D0/D1 串口

C07A 原理图里，`PA10/PA11` 接到了板载 `CH9102F` USB 转串口：

- `PA10` 连接 CH9102F 的 `RXD`
- `PA11` 连接 CH9102F 的 `TXD`

也就是说，C07A 默认已经把 G3507 的 UART0 留给 USB 下载/调试串口使用，并没有在 H1/H2 上直接给出 `PA10/PA11`。

因此：

- 默认不建议把 NANO 转 UNO 板的 `D0/D1` 再飞线出来接 HC-05。
- 如果必须接 HC-05 蓝牙串口，优先使用 C07A 板载 USB 串口以外的另一组 UART，并重新分配 H1/H2 上的引脚。
- 例如 `PB2/PB3` 可复用为 `UART3_TX/UART3_RX`，但本推荐表已把它们分给电机 PWM；要用蓝牙就需要重排电机引脚并同步改代码。

## 供电注意

- C07A 核心板可以从 USB-C 获得 5V，并通过板载稳压得到 3.3V。
- NANO 转 UNO 板上的 `VIN` 不要接 C07A，除非你非常确认转接板电源路径。
- 推荐先只把 C07A 的 `3V3` 接到 Nano 插座 `3V3`，把 C07A 的 `GND` 接到 Nano 插座 `GND`。
- 如果 NANO 转 UNO 板或小车外设需要 UNO 侧 `5V` 电源轨，可以把 C07A 的 `H1-1 5V` 接到 Nano 插座 `+5V`，但这只用于供电，不代表 5V 信号可以直接输入 G3507。
- 电机电池正极只接电机驱动 `VM`/`Motor VIN`，不要接到 C07A 的 `3V3` 或 `5V`。
- C07A、小车传感器、电机驱动、电机电池负极必须共地。

## 电平注意

- MSPM0G3507 工作电源范围是 1.62V 到 3.6V，C07A 核心板是 3.3V 逻辑。
- TI 数据手册写的是 `Two 5V-tolerant open drain IOs`，不是所有 IO 都能直接接 5V。
- PA0/PA1 是这两个 5V-tolerant open-drain IO，适合作 I2C 的 SDA/SCL，但 MPU6050 仍建议使用 3.3V 供电和 3.3V 上拉。
- LF04、编码器、HC-SR04 Echo 等如果输出 5V，高电平进 C07A 前应分压或电平转换到 3.3V。
- 电机驱动输入必须能识别 3.3V 高电平；如果不能识别，需要换支持 3.3V 逻辑的驱动或加电平转换。

## 代码需要同步改

上面的推荐飞线表和当前项目原始 Arduino 逻辑引脚一致，但 G3507 真实端口已经变成 C07A 的 H1/H2 端口。后续代码应按下列映射实现 `platform_hw_*`：

| 逻辑功能 | G3507 信号 |
| --- | --- |
| 巡线 S1/S2/S3/S4 | PA27 / PA26 / PA25 / PA24 |
| MPU6050 I2C | PA0 SDA / PA1 SCL |
| 左电机 PWM/GPIO | PA7 / PB2 |
| 右电机 PWM/GPIO | PB3 / PA17 |
| 编码器中断 B 相 | PA12 / PA8 |
| 编码器 A 相 | PA9 / PA13 |
| 启动按钮 | PB17 |
| HC-SR04 可选 | PA22 / PA15 |

当前 `src/platform/platform_mspm0.c` 里的 `platform_hw_*` 仍是 weak 空实现，需要按这个表做真实 GPIO、PWM、I2C 和中断配置。

## 上电前检查

- C07A 的 `3V3` 没有和外部 5V 短接。
- C07A 的 GND、NANO 转 UNO 板 GND、小车传感器 GND、电机驱动 GND、电池负极已经共地。
- 电机电源没有接到 C07A 的 `3V3` 或 `5V`。
- 输入 C07A 的高电平不超过 3.3V，除非明确确认该脚可承受 5V。
- 先不接电机，只接传感器和按钮测试电平；确认软件读数正确后再接电机驱动。
