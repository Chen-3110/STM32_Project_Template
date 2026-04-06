#include "stepper_motor.h"
#include "tim.h"
#include "soft_timer.h"
#include <stdlib.h>
#include <math.h>

// 外部UART句柄声明
extern UART_HandleTypeDef huart1;

// 步进每毫米转换系数（与main.c保持一致）
#define STEPS_PER_MM_X  200.0f
#define STEPS_PER_MM_Y  200.0f
#define STEPS_PER_MM_Z  400.0f

// 全局变量定义
Plotter_Job_t g_plotter_job = {0};
static Plotter_Hardware_t hw_config;

// 当前位置（步数）
static int32_t current_pos_steps_x = 0;
static int32_t current_pos_steps_y = 0;
static int32_t current_pos_steps_z = 0;

// S型曲线默认配置
static S_Curve_Config_t s_curve_config = {
    .accel_ratio = 0.33f,  // 加速段占总步数33%
    .jerk_ratio = 1.0f,    // 加加速度系数
    .phase_ratios = {0.2f, 0.3f, 0.5f, 0.0f, 0.5f, 0.3f, 0.2f} // 各阶段比例
};

// Z轴运动状态（已在 stepper_motor.h 中定义）
Z_Axis_State_t z_axis_state = {0};

#ifndef PI
#define PI 3.1415926535f
#endif

/**
 * @brief 写字机系统参数化初始化
 * @param hw_config_param 硬件配置结构体指针，包含所有步进电机的GPIO引脚定义
 * 
 * 功能说明：
 * 1. 保存硬件配置信息到静态变量
 * 2. 初始化全局任务状态结构体
 * 3. 为后续的直线插补运动准备初始状态
 * 
 * 初始化时机：系统启动时调用一次
 */
void Plotter_Init(Plotter_Hardware_t *hw_config_param) {
    // 1. 硬件配置保存 - 将传入的引脚配置保存到静态变量供后续使用
    hw_config = *hw_config_param;
    
    // 2. 全局任务结构初始化 - 重置所有运动控制参数
    g_plotter_job.is_busy = 0;           // 任务忙标志：0=空闲，1=运行中
    g_plotter_job.current_step = 0;      // 当前已执行步数
    g_plotter_job.total_steps = 0;       // 总步数（Bresenham算法使用）
    g_plotter_job.err = 0;               // Bresenham算法误差累积项
    g_plotter_job.dx = 0;                // X轴位移量（绝对值）
    g_plotter_job.dy = 0;                // Y轴位移量（绝对值）
    g_plotter_job.dir_x = 1;             // X轴方向：1=正方向，-1=负方向
    g_plotter_job.dir_y = 1;             // Y轴方向：1=正方向，-1=负方向
    g_plotter_job.current_arr = 0;       // 当前定时器ARR值（控制脉冲频率）
    g_plotter_job.start_arr = 0;         // 起始ARR值（加速起始频率）
    g_plotter_job.peak_arr = 0;          // 峰值ARR值（目标运行频率）
    g_plotter_job.accel_steps = 0;       // 加速段步数（T型加减速）
    g_plotter_job.arr_step = 0;          // 每步ARR变化量（加速斜率）
}


/**
 * @brief 启动直线插补运动（Bresenham算法 + T型加减速）
 * @param tx 目标X坐标（脉冲数，可为负值表示反向）
 * @param ty 目标Y坐标（脉冲数，可为负值表示反向）
 * @param speed_hz 目标速度（Hz，范围400-12000）
 * 
 * 算法原理：
 * 1. 使用Bresenham直线算法进行两轴同步插补
 * 2. 采用T型加减速曲线实现平滑启停
 * 3. 通过定时器中断精确控制脉冲频率
 */
void Plotter_StartLine(int32_t tx, int32_t ty, uint16_t speed_hz) {
    // 1. 检查任务状态 - 防止任务冲突
    if (g_plotter_job.is_busy) return;

    // 2. 输入参数验证
    if (speed_hz < 400) speed_hz = 400;
    if (speed_hz > 12000) speed_hz = 12000;
    
    // 3. 空间与方向计算（Bresenham算法预处理）
    g_plotter_job.dx = abs(tx);           // X轴绝对位移量
    g_plotter_job.dy = abs(ty);           // Y轴绝对位移量
    g_plotter_job.dir_x = (tx >= 0) ? 1 : -1;  // X轴方向：1为正，-1为负
    g_plotter_job.dir_y = (ty >= 0) ? 1 : -1;  // Y轴方向：1为正，-1为负
    
    // Bresenham算法：总步数取dx和dy中的较大值
    g_plotter_job.total_steps = (g_plotter_job.dx > g_plotter_job.dy) ? g_plotter_job.dx : g_plotter_job.dy;
    
    // 检查是否为无效运动（零位移）
    if (g_plotter_job.total_steps == 0) return;
    
    // 边界条件处理：总步数过少时调整加速段
    if (g_plotter_job.total_steps < 20) {
        g_plotter_job.accel_steps = g_plotter_job.total_steps / 4; // 减少加速段比例
    } else {
        g_plotter_job.accel_steps = g_plotter_job.total_steps / 5; // 加速段占总步数的20%
    }

    // 4. 物理方向设置 - 通过GPIO控制电机旋转方向
    if (g_plotter_job.dir_x == 1)
        hw_config.x_dir.port->BSRR = hw_config.x_dir.pin;      // 设置X轴正方向（高电平）
    else
        hw_config.x_dir.port->BSRR = (uint32_t)hw_config.x_dir.pin << 16; // 设置X轴反方向（低电平）
    
    if (g_plotter_job.dir_y == 1)
        hw_config.y_dir.port->BSRR = hw_config.y_dir.pin;      // 设置Y轴正方向（高电平）
    else
        hw_config.y_dir.port->BSRR = (uint32_t)hw_config.y_dir.pin << 16; // 设置Y轴反方向（低电平）

    // 5. S型加减速初始化
    // 频率范围：400Hz（低速）~ 12kHz（高速）
    // ARR计算公式：ARR = (1MHz基础频率 / 目标频率) - 1
    g_plotter_job.start_arr = 2500 - 1;      // 起始频率400Hz：1MHz/400 = 2500
    g_plotter_job.peak_arr = (1000000 / speed_hz) - 1; // 目标频率对应的ARR值
    
    // 限制ARR范围，确保频率在有效范围内
    if (g_plotter_job.peak_arr < 83) g_plotter_job.peak_arr = 83;   // 上限12kHz：1MHz/12000 ≈ 83
    if (g_plotter_job.peak_arr > 2499) g_plotter_job.peak_arr = 2499; // 下限400Hz
    
    // 6. 计算S型曲线参数（7段曲线）
    // 使用配置参数计算总加速段步数
    uint32_t total_accel_steps = (uint32_t)(g_plotter_job.total_steps * s_curve_config.accel_ratio);
    
    // 安全检查：如果总加速步数小于10，回退到简单的T型加减速或匀速运动
    uint8_t use_s_curve = 1;
    if (total_accel_steps < 10) {
        // 不执行S型曲线计算，使用匀速运动（直接以峰值频率运行）
        use_s_curve = 0;
        total_accel_steps = 0;
    }
    
    if (use_s_curve) {
        // 分配各阶段步数（使用配置的比例）
        // 阶段0: 加加速段
        // 阶段1: 匀加速段
        // 阶段2: 减加速段
        // 阶段3: 匀速段
        // 阶段4: 加减速段
        // 阶段5: 匀减速段
        // 阶段6: 减减速段
        g_plotter_job.phase_steps[0] = total_accel_steps * s_curve_config.phase_ratios[0];
        g_plotter_job.phase_steps[1] = total_accel_steps * s_curve_config.phase_ratios[1];
        g_plotter_job.phase_steps[2] = total_accel_steps * s_curve_config.phase_ratios[2];
        g_plotter_job.phase_steps[3] = g_plotter_job.total_steps - 2 * total_accel_steps; // 匀速段
        g_plotter_job.phase_steps[4] = total_accel_steps * s_curve_config.phase_ratios[4];
        g_plotter_job.phase_steps[5] = total_accel_steps * s_curve_config.phase_ratios[5];
        g_plotter_job.phase_steps[6] = total_accel_steps * s_curve_config.phase_ratios[6];
        
        // 计算加加速度（jerk）参数，应用jerk_ratio系数，转换为Q16定点数
        // jerk_q16 = (start_arr - peak_arr) * 65536 / (total_accel_steps * total_accel_steps) * jerk_ratio
        // 使用int64_t中间计算防止32位溢出
        if (total_accel_steps > 0) {
            int32_t arr_diff = (int32_t)(g_plotter_job.start_arr - g_plotter_job.peak_arr);
            int64_t denominator = (int64_t)total_accel_steps * total_accel_steps;
            // 使用int64_t进行乘法，防止溢出
            int64_t numerator = (int64_t)arr_diff * 65536;
            int64_t jerk_value = numerator / denominator;
            // 应用jerk_ratio系数并转换为int32_t
            g_plotter_job.jerk_q16 = (int32_t)(jerk_value * s_curve_config.jerk_ratio);
        } else {
            g_plotter_job.jerk_q16 = 0;
        }
    } else {
        // 不使用S型曲线，设置所有阶段为匀速（峰值频率）
        for (int i = 0; i < 7; i++) {
            g_plotter_job.phase_steps[i] = 0;
        }
        g_plotter_job.phase_steps[3] = g_plotter_job.total_steps; // 整个运动都是匀速段
        g_plotter_job.jerk_q16 = 0;
    }
    
    // 预计算阶段边界累积步数
    g_plotter_job.phase_boundaries[0] = g_plotter_job.phase_steps[0];
    for (int i = 1; i < 7; i++) {
        g_plotter_job.phase_boundaries[i] = g_plotter_job.phase_boundaries[i-1] + g_plotter_job.phase_steps[i];
    }
    
    // 初始化阶段参数
    g_plotter_job.accel_phase = 0;
    g_plotter_job.current_phase_step = 0;

    // 7. 初始化运动状态参数
    g_plotter_job.current_arr = g_plotter_job.start_arr;  // 当前ARR从起始值开始
    g_plotter_job.current_step = 0;                       // 当前步数清零
    g_plotter_job.err = 0;                                // Bresenham误差项清零
    g_plotter_job.is_busy = 1;                            // 设置忙标志，开始任务
    
    // 8. 启动TIM6定时器 - 开始脉冲生成
    __HAL_TIM_SET_AUTORELOAD(&htim6, g_plotter_job.current_arr);  // 设置初始ARR值
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);                // 清除更新标志
    HAL_TIM_Base_Start_IT(&htim6);                               // 启动定时器中断
}

/**
 * @brief 停止插补任务
 */
void Plotter_Stop(void) {
    g_plotter_job.is_busy = 0;
    HAL_TIM_Base_Stop_IT(&htim6);
}

/**
 * @brief Z轴控制
 * @param target_z 目标Z坐标（脉冲数，正值为上升，负值为下降）
 * @param speed_hz 目标速度（Hz，范围400-12000）
 */
void Plotter_SetZ(int32_t target_z, uint16_t speed_hz) {
    // 1. 检查Z轴是否正在运动
    if (z_axis_state.is_busy) {
        return;
    }
    
    // 2. 输入参数验证
    if (speed_hz < 400) speed_hz = 400;
    if (speed_hz > 12000) speed_hz = 12000;
    
    // 3. 计算目标步数（绝对值）
    z_axis_state.target_steps = abs(target_z);
    if (z_axis_state.target_steps == 0) {
        return; // 零位移，直接返回
    }
    
    // 4. 方向控制：根据target_z的正负设置PB1 (Z_Dir)的电平
    if (target_z >= 0) {
        // 正方向
        hw_config.z_dir.port->BSRR = hw_config.z_dir.pin;
        z_axis_state.direction = 1;
    } else {
        // 负方向
        hw_config.z_dir.port->BSRR = (uint32_t)hw_config.z_dir.pin << 16;
        z_axis_state.direction = 0;
    }
    
    // 5. 计算ARR值（基于1MHz基准频率）
    // ARR计算公式：ARR = (1MHz基础频率 / 目标频率) - 1
    uint16_t arr = (1000000 / speed_hz) - 1;
    
    // 限制ARR范围，确保频率在有效范围内
    if (arr < 83) arr = 83;       // 上限12kHz：1MHz/12000 ≈ 83
    if (arr > 2499) arr = 2499;   // 下限400Hz
    
    // 6. 配置TIM3
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, arr / 2); // 50%占空比
    
    // 7. 初始化Z轴状态
    z_axis_state.current_step = 0;
    z_axis_state.is_busy = 1;
    
    // 8. 启动TIM3 PWM输出（TIM3_CH3对应PB0）
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    
    // 9. 开启TIM3更新中断用于步数计数
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

/**
 * @brief 中断回调：执行脉冲输出
 * 包含Bresenham引脚翻转、脉宽延时以及current_arr的加减速更新逻辑
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // 0. 硬限位紧急停机检测 (X:PE7, Y:PE8, Z:PE9) - 适用于所有定时器
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_RESET ||  // X轴限位
        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_RESET ||  // Y轴限位
        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET)    // Z轴限位
    {
        Plotter_Stop();
        g_plotter_job.is_busy = 0;
        
        // 同时停止Z轴运动
        if (z_axis_state.is_busy) {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
            z_axis_state.is_busy = 0;
        }
        
        // 发送具体的限位错误信息
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_RESET) {
            static const uint8_t error_msg[] = "Error: X-Limit Triggered\n";
            HAL_UART_Transmit(&huart1, error_msg, sizeof(error_msg)-1, 10);
        } else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_RESET) {
            static const uint8_t error_msg[] = "Error: Y-Limit Triggered\n";
            HAL_UART_Transmit(&huart1, error_msg, sizeof(error_msg)-1, 10);
        } else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET) {
            static const uint8_t error_msg[] = "Error: Z-Limit Triggered\n";
            HAL_UART_Transmit(&huart1, error_msg, sizeof(error_msg)-1, 10);
        }
        return;
    }
    
    if (htim->Instance == TIM6) {
        // 1. 检查任务状态
        if (!g_plotter_job.is_busy) {
            HAL_TIM_Base_Stop_IT(&htim6);
            return;
        }

        // 2. Bresenham算法步进 - 使用BSRR寄存器直接操作引脚
        uint8_t need_x_step = 0;
        uint8_t need_y_step = 0;

        if (g_plotter_job.dx >= g_plotter_job.dy) {
            // 主轴为X轴
            need_x_step = 1;
            g_plotter_job.err += g_plotter_job.dy;
            
            if (g_plotter_job.err >= g_plotter_job.dx) {
                g_plotter_job.err -= g_plotter_job.dx;
                need_y_step = 1;
            }
        } else {
            // 主轴为Y轴
            need_y_step = 1;
            g_plotter_job.err += g_plotter_job.dx;
            
            if (g_plotter_job.err >= g_plotter_job.dy) {
                g_plotter_job.err -= g_plotter_job.dy;
                need_x_step = 1;
            }
        }

        // 拉高需要步进的引脚
        if (need_x_step) {
            hw_config.x_step.port->BSRR = hw_config.x_step.pin;
        }
        if (need_y_step) {
            hw_config.y_step.port->BSRR = hw_config.y_step.pin;
        }

        // 3. 脉冲宽度延时（确保步进电机能识别脉冲）
        // 使用DWT微秒延时，稳定2微秒，确保驱动器（如TB6600）能可靠识别
        Delay_us(2);

        // 4. 拉低所有STEP引脚（脉冲下降沿）
        hw_config.x_step.port->BSRR = (uint32_t)hw_config.x_step.pin << 16;
        hw_config.y_step.port->BSRR = (uint32_t)hw_config.y_step.pin << 16;
        
        // 5. 更新步数计数器
        g_plotter_job.current_step++;
        
        // 更新当前位置（步数）
        if (need_x_step) {
            current_pos_steps_x += g_plotter_job.dir_x;
        }
        if (need_y_step) {
            current_pos_steps_y += g_plotter_job.dir_y;
        }
        
        // 6. S型加减速控制（7段曲线）- 优化版本
        // 使用预计算的阶段边界快速确定当前阶段
        uint8_t current_phase = g_plotter_job.accel_phase;
        
        // 如果当前步数超过当前阶段边界，更新到下一个阶段
        if (g_plotter_job.current_step >= g_plotter_job.phase_boundaries[current_phase]) {
            if (current_phase < 6) {
                current_phase++;
                g_plotter_job.accel_phase = current_phase;
                g_plotter_job.current_phase_step = 0;
            }
        }
        
        // 根据当前阶段计算ARR变化（使用定点数运算）
        switch (current_phase) {
            case 0: // 加加速段
                // arr = start_arr - (jerk_q16 * t²) >> 16
                {
                    uint32_t t_sq = g_plotter_job.current_phase_step * g_plotter_job.current_phase_step;
                    int32_t adjustment = (g_plotter_job.jerk_q16 * t_sq) >> 16;
                    g_plotter_job.current_arr = g_plotter_job.start_arr - (uint16_t)adjustment;
                }
                g_plotter_job.current_phase_step++;
                break;
                
            case 1: // 匀加速段
                // arr = arr - 固定步长（预计算）
                {
                    uint16_t step_size = (g_plotter_job.start_arr - g_plotter_job.peak_arr) /
                                        (g_plotter_job.phase_steps[1] + g_plotter_job.phase_steps[2]);
                    if (step_size == 0) step_size = 1;
                    if (g_plotter_job.current_arr > g_plotter_job.peak_arr + step_size) {
                        g_plotter_job.current_arr -= step_size;
                    } else {
                        g_plotter_job.current_arr = g_plotter_job.peak_arr;
                    }
                }
                g_plotter_job.current_phase_step++;
                break;
                
            case 2: // 减加速段
                // arr = peak_arr + (jerk_q16 * (剩余步数)²) >> 16
                {
                    uint16_t remaining_steps = g_plotter_job.phase_steps[2] - g_plotter_job.current_phase_step;
                    uint32_t t_sq = remaining_steps * remaining_steps;
                    int32_t adjustment = (g_plotter_job.jerk_q16 * t_sq) >> 16;
                    g_plotter_job.current_arr = g_plotter_job.peak_arr + (uint16_t)adjustment;
                }
                g_plotter_job.current_phase_step++;
                break;
                
            case 3: // 匀速段
                // 保持峰值频率
                g_plotter_job.current_arr = g_plotter_job.peak_arr;
                // 不需要更新current_phase_step
                break;
                
            case 4: // 加减速段
                // arr = peak_arr + (jerk_q16 * t²) >> 16
                {
                    uint32_t t_sq = g_plotter_job.current_phase_step * g_plotter_job.current_phase_step;
                    int32_t adjustment = (g_plotter_job.jerk_q16 * t_sq) >> 16;
                    g_plotter_job.current_arr = g_plotter_job.peak_arr + (uint16_t)adjustment;
                }
                g_plotter_job.current_phase_step++;
                break;
                
            case 5: // 匀减速段
                // arr = arr + 固定步长（预计算）
                {
                    uint16_t step_size = (g_plotter_job.start_arr - g_plotter_job.peak_arr) /
                                        (g_plotter_job.phase_steps[5] + g_plotter_job.phase_steps[6]);
                    if (step_size == 0) step_size = 1;
                    if (g_plotter_job.current_arr < g_plotter_job.start_arr - step_size) {
                        g_plotter_job.current_arr += step_size;
                    } else {
                        g_plotter_job.current_arr = g_plotter_job.start_arr;
                    }
                }
                g_plotter_job.current_phase_step++;
                break;
                
            case 6: // 减减速段
                // arr = start_arr - (jerk_q16 * (剩余步数)²) >> 16
                {
                    uint16_t remaining_steps = g_plotter_job.phase_steps[6] - g_plotter_job.current_phase_step;
                    uint32_t t_sq = remaining_steps * remaining_steps;
                    int32_t adjustment = (g_plotter_job.jerk_q16 * t_sq) >> 16;
                    g_plotter_job.current_arr = g_plotter_job.start_arr - (uint16_t)adjustment;
                }
                g_plotter_job.current_phase_step++;
                break;
        }
        
        // 限制ARR范围，确保频率在有效范围内
        if (g_plotter_job.current_arr < g_plotter_job.peak_arr) {
            g_plotter_job.current_arr = g_plotter_job.peak_arr;
        }
        if (g_plotter_job.current_arr > g_plotter_job.start_arr) {
            g_plotter_job.current_arr = g_plotter_job.start_arr;
        }
        
        // 7. 更新TIM6的ARR值
        __HAL_TIM_SET_AUTORELOAD(&htim6, g_plotter_job.current_arr);
        
        // 8. 检查是否到达终点
        if (g_plotter_job.current_step >= g_plotter_job.total_steps) {
            g_plotter_job.is_busy = 0;
            HAL_TIM_Base_Stop_IT(&htim6);
        }
    }
    else if (htim->Instance == TIM3) {
        // TIM3中断：Z轴步数计数
        if (!z_axis_state.is_busy) {
            // Z轴不忙，停止中断
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
            return;
        }
        
        // 增加步数计数
        z_axis_state.current_step++;
        
        // 更新Z轴当前位置（步数）
        if (z_axis_state.direction == 1) {
            current_pos_steps_z++;
        } else {
            current_pos_steps_z--;
        }
        
        // 检查是否到达目标步数
        if (z_axis_state.current_step >= z_axis_state.target_steps) {
            // 到达目标步数，停止PWM输出
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
            z_axis_state.is_busy = 0;
        }
    }
}

// 以下为旧版兼容函数（保持原有功能）

/**
 * @brief 初始化电机绑定硬件
 */
void Stepper_Init(StepperMotor_t *motor, TIM_HandleTypeDef *htim, uint32_t ch, GPIO_TypeDef *port, uint16_t pin) {
    motor->htim = htim;
    motor->channel = ch;
    motor->dir_port = port;
    motor->dir_pin = pin;
    
    motor->isRunning = 0;
    motor->state = 1;
    motor->current_dir = 1; // 默认方向
}

/**
 * @brief 软启动/动态调速/V型换向 API (支持运行中直接调速和换向)
 */
void Stepper_Start_Soft_Freq(StepperMotor_t *motor, uint8_t dir, float start_freq, float target_freq, uint32_t accel_time_ms) {
    // 1. 频率转 ARR，并限制下限
    if (start_freq < 400.0f) start_freq = 400.0f;
    if (target_freq < 400.0f) target_freq = 400.0f;

    uint16_t s_arr = (uint16_t)(24000000.0f / start_freq) - 1;
    uint16_t t_arr = (uint16_t)(24000000.0f / target_freq) - 1;

    // 2. 时间折算为任务执行次数
    uint32_t total_ticks = accel_time_ms / 2;
    if (total_ticks == 0) total_ticks = 1;
    
    // 3. 状态机核心分发
    if (motor->isRunning == 0) {
        // 【情况 A：完全静止时的冷启动】
        motor->current_dir = dir;
        // 只有冷启动时，才立刻操作物理引脚
        if(dir == 1) motor->dir_port->BSRR = motor->dir_pin;
        else motor->dir_port->BSRR = (uint32_t)motor->dir_pin << 16;
        
        motor->current_arr = s_arr;
        motor->target_arr = t_arr;
        
        // 计算步长
        uint16_t diff = (s_arr > t_arr) ? (s_arr - t_arr) : (t_arr - s_arr);
        motor->step_change = diff / total_ticks;
        if (motor->step_change == 0) motor->step_change = 1;

        motor->state = 1;
        motor->isRunning = 1;
        motor->last_tick = HAL_GetTick();
        
        __HAL_TIM_SET_AUTORELOAD(motor->htim, motor->current_arr);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->current_arr / 2);
        HAL_TIM_PWM_Start(motor->htim, motor->channel);
    } 
    else if (motor->current_dir != dir) {
        // 【情况 B：运行中突发反转指令 (V型换向)】
        motor->state = 2;
        motor->pending_dir = dir;
        motor->pending_target_arr = t_arr;
        motor->target_arr = 60000;
        
        // 计算减速步长
        uint16_t diff = (60000 > motor->current_arr) ? (60000 - motor->current_arr) : 0;
        motor->step_change = diff / total_ticks;
        if (motor->step_change == 0) motor->step_change = 1;
    } 
    else {
        // 【情况 C：运行中同向变速】
        motor->target_arr = t_arr;
        motor->state = 1;
        
        // 计算变速步长
        uint16_t diff = (motor->current_arr > t_arr) ? (motor->current_arr - t_arr) : (t_arr - motor->current_arr);
        motor->step_change = diff / total_ticks;
        if (motor->step_change == 0) motor->step_change = 1;
    }
}

/**
 * @brief 停止电机
 */
void Stepper_Stop(StepperMotor_t *motor) {
    motor->isRunning = 0;
    motor->state = 1;
    HAL_TIM_PWM_Stop(motor->htim, motor->channel);
}

/**
 * @brief 发起软停止指令 (平滑刹车)
 */
void Stepper_Stop_Soft(StepperMotor_t *motor, uint32_t decel_time_ms) {
    if (motor->isRunning == 0) return;

    motor->state = 3;
    motor->target_arr = 60000;
    
    uint32_t total_ticks = decel_time_ms / 2;
    if (total_ticks == 0) total_ticks = 1;
    
    uint16_t diff = (60000 > motor->current_arr) ? (60000 - motor->current_arr) : 0;
    motor->step_change = diff / total_ticks;
    if (motor->step_change == 0) motor->step_change = 1;
}

/**
 * @brief 后台管家任务
 */
void Stepper_Run_Task(StepperMotor_t *motor) {
    if (motor->isRunning == 0) return;

    uint32_t current_tick = HAL_GetTick();
    if (current_tick - motor->last_tick < 2) return;
    motor->last_tick = current_tick;

    // 1. 无脑执行当前的加减速任务
    if (motor->current_arr > motor->target_arr) {
        if (motor->current_arr - motor->target_arr <= motor->step_change) motor->current_arr = motor->target_arr;
        else motor->current_arr -= motor->step_change;
    } else if (motor->current_arr < motor->target_arr) {
        if (motor->target_arr - motor->current_arr <= motor->step_change) motor->current_arr = motor->target_arr;
        else motor->current_arr += motor->step_change;
    }

    // 2. 换向拦截器
    if (motor->state == 2 && motor->current_arr >= 60000) {
        if(motor->pending_dir == 1) motor->dir_port->BSRR = motor->dir_pin;
        else motor->dir_port->BSRR = (uint32_t)motor->dir_pin << 16;
        
        motor->current_dir = motor->pending_dir;
        motor->target_arr = motor->pending_target_arr;
        motor->state = 1;
    }
    if (motor->state == 3 && motor->current_arr >= 60000) {
        Stepper_Stop(motor);
        return;
    }

    // 3. 刷新硬件寄存器
    __HAL_TIM_SET_AUTORELOAD(motor->htim, motor->current_arr);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->current_arr / 2);
}

/**
 * @brief 一键软换向
 */
void Stepper_Reverse_Soft(StepperMotor_t *motor, uint32_t reverse_decel_time_ms) {
    if (motor->isRunning == 0) {
        motor->current_dir = !(motor->current_dir);
        if(motor->current_dir == 1) motor->dir_port->BSRR = motor->dir_pin;
        else motor->dir_port->BSRR = (uint32_t)motor->dir_pin << 16;
        return;
    }

    if (motor->state == 2) return;

    motor->state = 2;
    motor->pending_dir = !(motor->current_dir);
    motor->pending_target_arr = motor->target_arr;
    motor->target_arr = 60000;

    uint32_t total_ticks = reverse_decel_time_ms / 2;
    if (total_ticks == 0) total_ticks = 1;
    
    uint16_t diff = (60000 > motor->current_arr) ? (60000 - motor->current_arr) : 0;
    motor->step_change = diff / total_ticks;
    if (motor->step_change == 0) motor->step_change = 1;
}

/**
 * @brief 按指定角度和总速度协同画斜线
 */
void Draw_Line_Angle(StepperMotor_t *motor_x, StepperMotor_t *motor_y, float angle_deg, float total_speed_hz, uint32_t accel_time_ms) {
    float angle_rad = angle_deg * PI / 180.0f;
    float raw_speed_x = total_speed_hz * cosf(angle_rad);
    float raw_speed_y = total_speed_hz * sinf(angle_rad);

    uint8_t dir_x = (raw_speed_x >= 0) ? 1 : 0;
    uint8_t dir_y = (raw_speed_y >= 0) ? 1 : 0;

    float abs_speed_x = fabsf(raw_speed_x);
    float abs_speed_y = fabsf(raw_speed_y);

    // 处理 X 轴
    if (abs_speed_x < 1.0f) {
        Stepper_Stop_Soft(motor_x, accel_time_ms);
    } else {
        Stepper_Start_Soft_Freq(motor_x, dir_x, 0, abs_speed_x, accel_time_ms);
    }

    // 处理 Y 轴
    if (abs_speed_y < 1.0f) {
        Stepper_Stop_Soft(motor_y, accel_time_ms);
    } else {
        Stepper_Start_Soft_Freq(motor_y, dir_y, 0, abs_speed_y, accel_time_ms);
    }
}

/**
 * @brief 设置S型曲线配置参数
 * @param config 配置结构体指针
 */
void Plotter_SetScurveConfig(S_Curve_Config_t *config) {
    if (config == NULL) return;
    
    // 复制配置参数
    s_curve_config.accel_ratio = config->accel_ratio;
    s_curve_config.jerk_ratio = config->jerk_ratio;
    
    // 复制各阶段比例
    for (int i = 0; i < 7; i++) {
        s_curve_config.phase_ratios[i] = config->phase_ratios[i];
    }
    
    // 验证参数有效性
    if (s_curve_config.accel_ratio < 0.1f) s_curve_config.accel_ratio = 0.1f;
    if (s_curve_config.accel_ratio > 0.5f) s_curve_config.accel_ratio = 0.5f;
    
    if (s_curve_config.jerk_ratio < 0.1f) s_curve_config.jerk_ratio = 0.1f;
    if (s_curve_config.jerk_ratio > 5.0f) s_curve_config.jerk_ratio = 5.0f;
}

/**
 * @brief 获取默认S型曲线配置
 * @param config 配置结构体指针，用于接收默认配置
 */
void Plotter_GetDefaultScurveConfig(S_Curve_Config_t *config) {
    if (config == NULL) return;
    
    // 设置默认配置
    config->accel_ratio = 0.33f;
    config->jerk_ratio = 1.0f;
    
    // 设置各阶段默认比例
    config->phase_ratios[0] = 0.2f;  // 加加速段
    config->phase_ratios[1] = 0.3f;  // 匀加速段
    config->phase_ratios[2] = 0.5f;  // 减加速段
    config->phase_ratios[3] = 0.0f;  // 匀速段（自动计算）
    config->phase_ratios[4] = 0.5f;  // 加减速段
    config->phase_ratios[5] = 0.3f;  // 匀减速段
    config->phase_ratios[6] = 0.2f;  // 减减速段
}

/**
 * @brief 获取当前位置（毫米）
 * @param x_mm X轴位置（毫米）输出指针
 * @param y_mm Y轴位置（毫米）输出指针
 * @param z_mm Z轴位置（毫米）输出指针
 */
void Plotter_GetCurrentPosition(float *x_mm, float *y_mm, float *z_mm) {
    if (x_mm) *x_mm = (float)current_pos_steps_x / STEPS_PER_MM_X;
    if (y_mm) *y_mm = (float)current_pos_steps_y / STEPS_PER_MM_Y;
    if (z_mm) *z_mm = (float)current_pos_steps_z / STEPS_PER_MM_Z;
}