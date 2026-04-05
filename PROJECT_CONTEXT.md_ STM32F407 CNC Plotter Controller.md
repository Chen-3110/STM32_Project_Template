# PROJECT_CONTEXT.md: STM32F407 CNC Plotter Controller

# 1. 核心目标 (Core Objective)

将现有的基于 STM32F103 的步进电机控制逻辑（Bresenham 插补 + T型加减速）迁移并适配至 STM32F407ZGT6 平台。实现通过 Web 上位机发送串口指令控制三轴联动绘图。

# 2. 硬件资源分配 (Hardware Resources)

|功能模块|引脚分配|资源/模式|备注|
|---|---|---|---|
|UART1 (PC通讯)|PA9(TX), PA10(RX)|DMA RX (Circular), DMA TX (Normal)|开启 IDLE 中断，波特率 115200|
|UART2 (Wi-Fi)|PA2(TX), PA3(RX)|预留 DMA 配置|暂不编写业务逻辑|
|TIM6 (运动心脏)|内部计数|1MHz，PSC=83, ARR=83 (12kHz IRQ)|运行 Bresenham 算法|
|TIM3 (Z轴脉冲)|PB0|TIM3_CH3 (PWM 模式)|用于 Z 轴独立步进控制|
|X轴控制|PB10(Step), PB11(Dir)|GPIO Output，High Speed|-|
|Y轴控制|PB12(Step), PB13(Dir)|GPIO Output，High Speed|-|
|Z轴方向|PB1|GPIO Output|控制 Z 轴升降方向|
|电机使能|PB2 (EN)|GPIO Output|低电平有效|
|限位开关|PE7(X), PE8(Y), PE9(Z)|GPIO Input|开启内部上拉 (Pull-up)|
# 3. 既有代码资产 (Legacy Assets)

项目 User/ 目录下已包含以下文件，禁止修改其内部函数定义，仅允许通过接口调用：

- soft_timer.c/h: 提供 DWT 微秒级精确延时 (DWT_Init, Delay_us)。

- stepper_motor.c/h:
        

    - Plotter_Job_t: 核心任务结构体。

    - Plotter_StartLine(tx, ty, speed): 启动插补。

    - HAL_TIM_PeriodElapsedCallback: 包含 Bresenham 逻辑。

# 4. 强制约束 (Mandatory Constraints)

## ⚠️ 编码位置约束 (Location Constraints)

仅允许在 /* USER CODE BEGIN ... */ 和 /* USER CODE END ... */ 注释对之间编写代码。禁止修改任何由 STM32CubeMX 生成的初始化函数（如 MX_GPIO_Init 等）。

## 🛠️ 构建系统约束 (Build System)

如果需要添加源文件或包含路径，只允许修改根目录下的 CMakeLists.txt。确保添加 User 目录到 include_directories 和 add_executable 的源文件列表中。

## 🚀 业务逻辑要求 (Business Logic)

- 通讯协议：采用“停-等”机制。解析格式 X100 Y200 Z10\n。

- 非阻塞解析：利用 UART IDLE (空闲中断) + DMA 接收。在中断中设置标志位，在 main.c 的 while(1) 中解析。

- 安全机制：运动开始前需拉低 PB2 (EN) 引脚。必须在 TIM6 中断中集成限位开关检测（PE7-9），检测到低电平时立即停止运动。

- 反馈：每一段运动任务完成后（g_plotter_job.is_busy 变为 0），必须通过 UART1 DMA 发送 OK\n。

# 5. 开发任务清单 (Task List for AI)

- 初始化：在 main.c 中调用 DWT_Init() 并配置 Plotter_Hardware_t 结构体绑定 PB 端口引脚。

- 串口解析器：编写命令解析逻辑，支持从 rx_buffer 中提取 X/Y/Z 目标坐标。

- 频率适配：确保 stepper_motor.c 移植后，ARR 计算基于 1MHz 基准。

- 中断完善：在 stm32f4xx_it.c 中处理 USART1_IRQHandler 的 IDLE 标志清除逻辑。
> （注：文档部分内容可能由 AI 生成）