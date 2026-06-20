# 串口引脚定义说明

本文档汇总当前项目中已经出现的串口相关引脚定义，并标出对应代码或文档位置。

## 结论

项目里目前只有一组串口引脚定义：HC-05 蓝牙串口模块接到逻辑引脚 `D0/D1`。

| 串口设备 | 模块引脚 | 项目逻辑引脚 | 方向说明 | 位置 |
| --- | --- | --- | --- | --- |
| HC-05 蓝牙串口模块 | TX | D0 | HC-05 的 TX 应接 MCU/开发板 RX；这里按 Arduino 风格逻辑引脚理解，`D0` 为 RX 侧。 | `docs/hardware.md:37-41` |
| HC-05 蓝牙串口模块 | RX | D1 | HC-05 的 RX 应接 MCU/开发板 TX；这里按 Arduino 风格逻辑引脚理解，`D1` 为 TX 侧。 | `docs/hardware.md:37-41` |

原始定义如下：

```text
HC-05 TX/RX -> D0/D1
```

## 代码位置

### 逻辑接线定义

- `docs/hardware.md:37-41`
  - 所属小节：`Optional hardware kept for debugging only`
  - 定义内容：`HC-05 TX/RX -> D0/D1`
  - 说明：这是项目里唯一明确写出的串口引脚接线。

### 硬件适配入口

- `src/platform/platform_mspm0.c:219-221`
  - 函数：`platform_hw_init()`
  - 说明：这是当前 MSPM0G3507 板级初始化弱函数。后续如果要真正启用 HC-05 串口，应在这里或其板级覆盖实现中配置 UART 外设、GPIO 复用和波特率。

- `include/platform/platform_hw.h:8-19`
  - 内容：当前所有 `platform_hw_*` 硬件适配接口声明。
  - 说明：此头文件目前没有 UART/串口收发接口声明。

### 非引脚定义引用

- `2024H.code-workspace:33-46`
  - 内容：推荐 VS Code 插件 `ms-vscode.vscode-serial-monitor`。
  - 说明：这是串口监视器工具推荐，不是串口硬件引脚定义。

## 当前未定义项

- 未指定 MSPM0G3507 的真实物理封装管脚。
- 未指定具体 UART 外设实例，例如 `UART0` 或 `UART1`。
- C 源码中没有 `D0/D1` 的 UART pin mux 配置。
- `platform_hw.h` 中没有串口读写 API。

## 检索范围

已检查以下内容：

- `2026Ti/2024H` 下全部源码、头文件、Markdown 文档、workspace 配置。
- `2026Ti/H题_自动行驶小车.pdf`，通过 `pdftotext` 检索 `串口`、`UART`、`USART`、`TX`、`RX`、`D0`、`D1`、`HC-05`、`蓝牙`，未发现额外串口引脚定义。

## 移植建议

1. 将逻辑 `D0` 映射到 MSPM0G3507 上具备 UART RX 功能的物理管脚。
2. 将逻辑 `D1` 映射到 MSPM0G3507 上具备 UART TX 功能的物理管脚。
3. 在 `platform_hw_init()` 或板级覆盖实现里配置 UART 外设和引脚复用。
4. 如果业务代码需要蓝牙串口收发，在 `include/platform/platform_hw.h` 增加明确的串口读写接口。
