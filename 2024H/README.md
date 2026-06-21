# 2024H 项目 — MSPM0G3507 自动小车 固件

本仓库为 2024 年大学生电子设计竞赛 H 题的自动小车固件示例。设计目标是将竞赛逻辑与板级实现分离，便于在不同硬件平台上移植与调参。

核心要点：

- 逻辑与硬件分离：`src/app`、`src/navigation`、`src/control` 为竞赛逻辑；`src/platform` 暴露板级适配点（weak 函数），方便移植。
- 所有可调参数集中在 `include/app_config.h`，用于快速调参与竞赛调优。

目录概览

- `include/`：公共头文件与参数（PID、速度、传感器阈值等）。
- `src/app/`：顶层应用状态机（启动、运行、完成）。
- `src/navigation/`：预定义路线（REQ1–REQ4）。
- `src/control/`：控制算法（PID、线跟踪、段管理、速度混合）。
- `src/platform/`：板级抽象与硬件 glue（包含 MSPM0 的实现和弱函数）。
- `src/system/`：最小 Cortex-M0+ 启动文件与中断向量。
- `docs/`：竞赛笔记与调参说明。
- `linker/`：MSPM0G3507 链接脚本。

硬件与依赖（概述）

- MCU：TI MSPM0G3507，80 MHz Cortex-M0+。
- 传感器：4 通道数字反射式线传感器、轮编码器、MPU6050（陀螺）。
- 驱动：双轮差速驱动，PWM 控制，编码器用于里程估计。

快速开始（功能级）

1. 修改参数：在 `include/app_config.h` 中设置车轮基线、编码器刻度、速度上限、线传感器阈值等。
2. 板级移植：在 `src/platform/platform_mspm0.c` 中实现或重写 `platform_hw_*` 系列弱函数，将其连接到具体的 GPIO、PWM、I2C、编码器读取与按键。
3. 编译与烧录：使用适合 MSPM0G3507 的交叉编译工具链（例如 arm-none-eabi-gcc），并用你们的烧录器/工具烧写固件。
4. 运行：上电后按 `start` 按键触发竞赛逻辑；通过拨码/开关选择路线。

主要模块说明与用法

**`src/platform/`（平台抽象）:**
- 角色：为上层提供传感器采样、时间基准与电机输出接口。
- 重要接口（在 `include/platform/platform.h`）:
	- `platform_init()`：硬件初始化（计时器、I2C、GPIO、PWM）。
	- `platform_millis()`：毫秒时钟，基于 SysTick。
	- `platform_read_sensors(PlatformSensors *s)`：填充线传感器、左右编码器、航向角。
	- `platform_set_motors(left_cm_s, right_cm_s)`：请求目标速度（cm/s），底层用 PI 控制转换为 PWM。
	- `platform_signal_waypoint(bool)` / `platform_signal_done(bool)`：用于提示（LED / 蜂鸣器）。
- 移植要点：实现 `platform_hw_*` 系列弱函数，使 `platform_read_sensors` 能读取真实引脚与外设，按需实现 I2C 读写用于 MPU6050。

**`src/control/`（控制层）:**
- 组件：`pid.c`（通用 PID），`motor_pi.c`（电机 PI 封装 + deadzone），`line_tracker.c`（基于 4 传感器的线跟踪 PID），`controller.c`（段管理与速度/偏航混合）。
- 用法：
	- 在 `controller_init()` 时传入路线计划与当前传感器，控制器会初始化 PID 并记录段起点状态。
	- 在定时回调（主循环的 `car_app_update`）中调用 `controller_update(controller, sensors, dt_s)`，控制器会根据当前段类型（直线/弧）计算左右目标速度并调用 `platform_set_motors()`。
- 调参：
	- 直线航向 PID（`APP_STRAIGHT_KP/KI/KD`）用于航向保持。
	- 线跟踪 PID（`APP_LINE_KP/KI/KD`）只在弧段中启用，用于基于线传感器修正弧行驶。
	- 电机 PI（`APP_LEFT_MOTOR_*` / `APP_RIGHT_MOTOR_*`）用于将速度误差变换为 PWM 输出并应用死区补偿。

**`src/navigation/`（路线）:**
- 格式：`RoutePlan` 由若干 `RouteSegment` 组成，每段定义类型（直线/左弧/右弧）、长度（cm）、速度（cm/s）、段终点标识符。
- 提供的计划：REQ1–REQ4（详见 `src/navigation/route.c`）。
- 切换方式：`platform_read_route_select()` -> `route_get_plan()`。

**`src/app/`（应用层）:**
- `car_app_init()`：状态初始化，清除提示。
- `car_app_update(now_ms)`：主状态机：等待启动 -> 运行 -> 完成。运行时会读取传感器并调用控制器接口；到达航点/完成路线时会触发提示。

配置与调试建议

- 先在台架上以低速验证运动学与编码器计数：检查 `APP_ENCODER_TICKS_PER_CM` 是否匹配硬件。
- 首先标定并验证陀螺（MPU6050）：`MPU6050` 静态偏置由 `platform` 层在 `platform_init()` 时采样并计算平均偏置。
- 调参顺序建议：
	1. 使左右电机在给定目标 PWM 下转速稳定（验证 PI 参数）。
	2. 调整直线航向 PID（`APP_STRAIGHT_*`），确保直线段航向稳定。
	3. 调整线跟踪 PID（`APP_LINE_*`），在弧段对线做出适当响应。
	4. 调整段速度与距离容差（`APP_DISTANCE_DONE_TOLERANCE_CM`）。

移植清单（最小项）

1. 在 `include/app_config.h` 设置好物理参数与阈值。
2. 实现 `src/platform/platform_mspm0.c` 中的 `platform_hw_*`：
	 - GPIO：线传感器读取、指示灯、蜂鸣器、按键、拨码开关。
	 - 编码器计数接口：`platform_hw_take_left_encoder_ticks()` / `platform_hw_take_right_encoder_ticks()`。
	 - PWM 输出：`platform_hw_set_left_pwm()` / `platform_hw_set_right_pwm()`。
	 - I2C：`platform_hw_i2c_read_regs()` / `platform_hw_i2c_write_reg()`（用于 MPU6050）。
3. 在硬件上验证 `platform_millis()` 的计时准确性和 SysTick 工作。

文件索引（快速参考）

- `include/app_config.h`：所有可调参数与常量。
- `include/platform/platform.h`：平台 API 与 `PlatformSensors` 结构。
- `src/platform/platform_mspm0.c`：当前默认的弱实现，移植点集中处。
- `src/control/controller.c`：段管理与电机目标生成。
- `src/navigation/route.c`：内置路线定义（REQ1–REQ4）。
- `src/app/car_app.c`：主状态机，串接平台与控制器。

如果你希望，我可以：

- 1) 基于当前代码生成一份更详细的 API 文档（按头文件逐条说明函数与数据结构），
- 2) 或者直接在 `src/platform/platform_mspm0.c` 中为你的目标开发板实现 `platform_hw_*`（请提供引脚分配）。

---

