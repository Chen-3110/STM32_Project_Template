#include "stepper_motor.h"
#include "tim.h"
#include <stdlib.h>

Plotter_Job_t g_plotter_job = {0};
Z_Axis_State_t z_axis_state = {0};
static Plotter_Hardware_t hw_config;

static S_Curve_Config_t s_curve_config = {
    .accel_ratio = 0.33f,  
    .jerk_ratio = 1.0f,    
    .phase_ratios = {0.2f, 0.3f, 0.5f, 0.0f, 0.5f, 0.3f, 0.2f} 
};

void Plotter_Init(Plotter_Hardware_t *hw_config_param) {
    hw_config = *hw_config_param;
    g_plotter_job.is_busy = 0;           
    g_plotter_job.limit_triggered = 0;
    // 初始化物理绝对坐标为0
    g_plotter_job.abs_step_x = 0;
    g_plotter_job.abs_step_y = 0;
    z_axis_state.abs_step_z = 0;
}

// 供主循环实时查询坐标的接口
void Plotter_GetCurrentPosition(float *x_mm, float *y_mm, float *z_mm) {
    *x_mm = (float)g_plotter_job.abs_step_x / STEPS_PER_MM_X;
    *y_mm = (float)g_plotter_job.abs_step_y / STEPS_PER_MM_Y;
    *z_mm = (float)z_axis_state.abs_step_z / STEPS_PER_MM_Z;
}

void Plotter_StartLine(int32_t tx, int32_t ty, uint16_t speed_hz) {
    if (g_plotter_job.is_busy) return;
    if (speed_hz < 400) speed_hz = 400;
    if (speed_hz > 12000) speed_hz = 12000;
    
    g_plotter_job.dx = abs(tx);           
    g_plotter_job.dy = abs(ty);           
    g_plotter_job.dir_x = (tx >= 0) ? 1 : -1;  
    g_plotter_job.dir_y = (ty >= 0) ? 1 : -1;  
    g_plotter_job.total_steps = (g_plotter_job.dx > g_plotter_job.dy) ? g_plotter_job.dx : g_plotter_job.dy;
    
    if (g_plotter_job.total_steps == 0) return;
    
    // 引脚方向
    if (g_plotter_job.dir_x == 1) hw_config.x_dir.port->BSRR = hw_config.x_dir.pin;
    else hw_config.x_dir.port->BSRR = (uint32_t)hw_config.x_dir.pin << 16; 
    
    if (g_plotter_job.dir_y == 1) hw_config.y_dir.port->BSRR = hw_config.y_dir.pin;
    else hw_config.y_dir.port->BSRR = (uint32_t)hw_config.y_dir.pin << 16; 

    g_plotter_job.start_arr = 2499;      
    g_plotter_job.peak_arr = (1000000 / speed_hz) - 1; 
    if (g_plotter_job.peak_arr < 83) g_plotter_job.peak_arr = 83;   
    
    uint32_t total_accel_steps = (uint32_t)(g_plotter_job.total_steps * s_curve_config.accel_ratio);
    
    // 安全防溢出处理：短位移时不计算复杂的 S 曲线
    if (total_accel_steps < 10) {
        total_accel_steps = g_plotter_job.total_steps / 2;
        g_plotter_job.jerk_q16 = 0; 
    } else {
        int64_t arr_diff = (int64_t)g_plotter_job.start_arr - g_plotter_job.peak_arr;
        int64_t denominator = (int64_t)total_accel_steps * total_accel_steps;
        g_plotter_job.jerk_q16 = (int32_t)((arr_diff * 65536LL / denominator) * s_curve_config.jerk_ratio);
    }
    
    g_plotter_job.phase_steps[0] = total_accel_steps * s_curve_config.phase_ratios[0];
    g_plotter_job.phase_steps[1] = total_accel_steps * s_curve_config.phase_ratios[1];
    g_plotter_job.phase_steps[2] = total_accel_steps * s_curve_config.phase_ratios[2];
    g_plotter_job.phase_steps[3] = g_plotter_job.total_steps - 2 * total_accel_steps; 
    g_plotter_job.phase_steps[4] = total_accel_steps * s_curve_config.phase_ratios[4];
    g_plotter_job.phase_steps[5] = total_accel_steps * s_curve_config.phase_ratios[5];
    g_plotter_job.phase_steps[6] = total_accel_steps * s_curve_config.phase_ratios[6];
    
    g_plotter_job.phase_boundaries[0] = g_plotter_job.phase_steps[0];
    for (int i = 1; i < 7; i++) g_plotter_job.phase_boundaries[i] = g_plotter_job.phase_boundaries[i-1] + g_plotter_job.phase_steps[i];
    
    g_plotter_job.accel_phase = 0;
    g_plotter_job.current_phase_step = 0;
    g_plotter_job.current_arr = g_plotter_job.start_arr;  
    g_plotter_job.current_step = 0;                       
    g_plotter_job.err = 0;                                
    g_plotter_job.is_busy = 1;                            
    
    __HAL_TIM_SET_AUTORELOAD(&htim6, g_plotter_job.current_arr);  
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);                
    HAL_TIM_Base_Start_IT(&htim6);                               
}

void Plotter_Stop(void) {
    g_plotter_job.is_busy = 0;
    HAL_TIM_Base_Stop_IT(&htim6);
}

void Plotter_SetZ(int32_t target_z, uint16_t speed_hz) {
    if (z_axis_state.is_busy) return;
    if (speed_hz < 400) speed_hz = 400;
    if (speed_hz > 12000) speed_hz = 12000;
    
    z_axis_state.target_steps = abs(target_z);
    if (z_axis_state.target_steps == 0) return; 
    
    if (target_z >= 0) {
        hw_config.z_dir.port->BSRR = hw_config.z_dir.pin;
        z_axis_state.direction = 1;
    } else {
        hw_config.z_dir.port->BSRR = (uint32_t)hw_config.z_dir.pin << 16;
        z_axis_state.direction = -1;
    }
    
    uint16_t arr = (1000000 / speed_hz) - 1;
    if (arr < 83) arr = 83;       
    if (arr > 2499) arr = 2499;   
    
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, arr / 2); 
    
    z_axis_state.current_step = 0;
    z_axis_state.is_busy = 1;
    
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // 硬限位检测
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7) == GPIO_PIN_RESET || 
        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_RESET ||  
        HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET)    
    {
        Plotter_Stop();
        g_plotter_job.limit_triggered = 1; // 仅置位，由 main 负责发送报错
        if (z_axis_state.is_busy) {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
            z_axis_state.is_busy = 0;
        }
        return;
    }
    
    if (htim->Instance == TIM6) {
        if (!g_plotter_job.is_busy) { HAL_TIM_Base_Stop_IT(&htim6); return; }

        if (g_plotter_job.dx >= g_plotter_job.dy) {
            hw_config.x_step.port->BSRR = hw_config.x_step.pin; 
            g_plotter_job.abs_step_x += g_plotter_job.dir_x; // 实时追踪绝对坐标
            g_plotter_job.err += g_plotter_job.dy;
            if (g_plotter_job.err >= g_plotter_job.dx) {
                g_plotter_job.err -= g_plotter_job.dx;
                hw_config.y_step.port->BSRR = hw_config.y_step.pin; 
                g_plotter_job.abs_step_y += g_plotter_job.dir_y;
            }
        } else {
            hw_config.y_step.port->BSRR = hw_config.y_step.pin; 
            g_plotter_job.abs_step_y += g_plotter_job.dir_y;
            g_plotter_job.err += g_plotter_job.dx;
            if (g_plotter_job.err >= g_plotter_job.dy) {
                g_plotter_job.err -= g_plotter_job.dy;
                hw_config.x_step.port->BSRR = hw_config.x_step.pin; 
                g_plotter_job.abs_step_x += g_plotter_job.dir_x;
            }
        }
        
        // 增宽脉冲延时至约 1.2us (168MHz下200个NOP)，确保驱动器不漏步
        for (volatile int i = 0; i < 200; i++) __NOP();
        
        hw_config.x_step.port->BSRR = (uint32_t)hw_config.x_step.pin << 16;
        hw_config.y_step.port->BSRR = (uint32_t)hw_config.y_step.pin << 16;
        
        g_plotter_job.current_step++;
        
        // --- 核心 S 曲线计算逻辑，为避免篇幅过长此处沿用你的匀速/S曲线切换机制 ---
        if (g_plotter_job.jerk_q16 > 0) { // 启用S曲线
            uint8_t current_phase = g_plotter_job.accel_phase;
            if (g_plotter_job.current_step >= g_plotter_job.phase_boundaries[current_phase]) {
                if (current_phase < 6) {
                    current_phase++;
                    g_plotter_job.accel_phase = current_phase;
                    g_plotter_job.current_phase_step = 0;
                }
            }
            switch (current_phase) {
                case 0: {
                    uint32_t t_sq = g_plotter_job.current_phase_step * g_plotter_job.current_phase_step;
                    g_plotter_job.current_arr = g_plotter_job.start_arr - (uint16_t)((g_plotter_job.jerk_q16 * t_sq) >> 16);
                } break;
                case 1: {
                    uint16_t step_size = (g_plotter_job.start_arr - g_plotter_job.peak_arr) / (g_plotter_job.phase_steps[1] + g_plotter_job.phase_steps[2] + 1);
                    if (g_plotter_job.current_arr > g_plotter_job.peak_arr + step_size) g_plotter_job.current_arr -= step_size;
                    else g_plotter_job.current_arr = g_plotter_job.peak_arr;
                } break;
                // ... 省略部分 case，沿用你的原有逻辑即可，注意防溢出
                case 3: g_plotter_job.current_arr = g_plotter_job.peak_arr; break;
                case 6: {
                    uint16_t remaining_steps = g_plotter_job.phase_steps[6] - g_plotter_job.current_phase_step;
                    g_plotter_job.current_arr = g_plotter_job.start_arr - (uint16_t)(((g_plotter_job.jerk_q16 * remaining_steps * remaining_steps)) >> 16);
                } break;
            }
            g_plotter_job.current_phase_step++;
        }

        if (g_plotter_job.current_arr < g_plotter_job.peak_arr) g_plotter_job.current_arr = g_plotter_job.peak_arr;
        if (g_plotter_job.current_arr > g_plotter_job.start_arr) g_plotter_job.current_arr = g_plotter_job.start_arr;
        __HAL_TIM_SET_AUTORELOAD(&htim6, g_plotter_job.current_arr);
        
        if (g_plotter_job.current_step >= g_plotter_job.total_steps) {
            g_plotter_job.is_busy = 0;
            HAL_TIM_Base_Stop_IT(&htim6);
        }
    }
    else if (htim->Instance == TIM3) {
        if (!z_axis_state.is_busy) { __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE); return; }
        
        z_axis_state.current_step++;
        z_axis_state.abs_step_z += z_axis_state.direction; // Z轴同步绝对步数
        
        if (z_axis_state.current_step >= z_axis_state.target_steps) {
            HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
            __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
            z_axis_state.is_busy = 0;
        }
    }
}