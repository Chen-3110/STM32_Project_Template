#include "stepper_motor.h"
#include "tim.h"
#include <stdlib.h>
#include <math.h>

// 全局变量定义
Plotter_Job_t g_plotter_job = {0};
static Plotter_Hardware_t hw_config;

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

    // 5. T型加速初始化（纯整数运算，避免浮点开销）
    // 频率范围：400Hz（低速）~ 12kHz（高速）
    // ARR计算公式：ARR = (1MHz基础频率 / 目标频率) - 1
    g_plotter_job.start_arr = 2500 - 1;      // 起始频率400Hz：1MHz/400 = 2500
    g_plotter_job.peak_arr = (1000000 / speed_hz) - 1; // 目标频率对应的ARR值
    
    // 限制ARR范围，确保频率在有效范围内
    if (g_plotter_job.peak_arr < 83) g_plotter_job.peak_arr = 83;   // 上限12kHz：1MHz/12000 ≈ 83
    if (g_plotter_job.peak_arr > 2499) g_plotter_job.peak_arr = 2499; // 下限400Hz
    
    // 6. 计算加速段参数（T型加速曲线）
    if (g_plotter_job.accel_steps > 0) {
        g_plotter_job.arr_step = (g_plotter_job.start_arr - g_plotter_job.peak_arr) / g_plotter_job.accel_steps;
    } else {
        g_plotter_job.arr_step = 0;
    }
    
    // 确保有加速过程（即使步数很少）
    if (g_plotter_job.arr_step == 0 && g_plotter_job.start_arr != g_plotter_job.peak_arr) {
        g_plotter_job.arr_step = 1;
    }

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
 * @brief Z轴控制（预留功能）
 */
void Plotter_SetZ(int32_t target_z, uint16_t speed_hz) {
    // 预留Z轴控制功能
    // 实现方式与X/Y轴类似
}

/**
 * @brief 中断回调：执行脉冲输出
 * 包含Bresenham引脚翻转、脉宽延时以及current_arr的加减速更新逻辑
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        // 1. 检查任务状态
        if (!g_plotter_job.is_busy) {
            HAL_TIM_Base_Stop_IT(&htim6);
            return;
        }

        // 2. Bresenham算法步进 - 使用BSRR寄存器直接操作引脚
        if (g_plotter_job.dx >= g_plotter_job.dy) {
            // 主轴为X轴
            hw_config.x_step.port->BSRR = hw_config.x_step.pin; // X轴脉冲上升沿
            g_plotter_job.err += g_plotter_job.dy;
            
            if (g_plotter_job.err >= g_plotter_job.dx) {
                g_plotter_job.err -= g_plotter_job.dx;
                hw_config.y_step.port->BSRR = hw_config.y_step.pin; // Y轴脉冲上升沿
            }
        } else {
            // 主轴为Y轴
            hw_config.y_step.port->BSRR = hw_config.y_step.pin; // Y轴脉冲上升沿
            g_plotter_job.err += g_plotter_job.dx;
            
            if (g_plotter_job.err >= g_plotter_job.dy) {
                g_plotter_job.err -= g_plotter_job.dy;
                hw_config.x_step.port->BSRR = hw_config.x_step.pin; // X轴脉冲上升沿
            }
        }
        
        // 3. 脉冲宽度延时（确保步进电机能识别脉冲）
        // 72个NOP在72MHz下约1μs，满足步进电机最小脉冲宽度要求
        for (volatile int i = 0; i < 72; i++) {
            __NOP();
        }
        
        // 4. 拉低所有STEP引脚（脉冲下降沿）
        hw_config.x_step.port->BSRR = (uint32_t)hw_config.x_step.pin << 16;
        hw_config.y_step.port->BSRR = (uint32_t)hw_config.y_step.pin << 16;
        
        // 5. 更新步数计数器
        g_plotter_job.current_step++;
        
        // 6. T型加减速控制
        if (g_plotter_job.current_step < g_plotter_job.accel_steps) {
            // 加速段：减小ARR（增加频率）
            if (g_plotter_job.current_arr > g_plotter_job.peak_arr) {
                g_plotter_job.current_arr -= g_plotter_job.arr_step;
                if (g_plotter_job.current_arr < g_plotter_job.peak_arr) {
                    g_plotter_job.current_arr = g_plotter_job.peak_arr;
                }
            }
        } else if (g_plotter_job.current_step > (g_plotter_job.total_steps - g_plotter_job.accel_steps)) {
            // 减速段：增加ARR（减小频率）
            if (g_plotter_job.current_arr < g_plotter_job.start_arr) {
                g_plotter_job.current_arr += g_plotter_job.arr_step;
                if (g_plotter_job.current_arr > g_plotter_job.start_arr) {
                    g_plotter_job.current_arr = g_plotter_job.start_arr;
                }
            }
        }
        // 匀速段：保持peak_arr不变
        
        // 7. 更新TIM6的ARR值
        __HAL_TIM_SET_AUTORELOAD(&htim6, g_plotter_job.current_arr);
        
        // 8. 检查是否到达终点
        if (g_plotter_job.current_step >= g_plotter_job.total_steps) {
            g_plotter_job.is_busy = 0;
            HAL_TIM_Base_Stop_IT(&htim6);
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