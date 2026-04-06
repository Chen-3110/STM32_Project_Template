#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "main.h"
#include <stdint.h>

// 统一将硬件参数定义在这里，防止 main 和 stepper_motor 出现双重定义
#define STEPS_PER_MM_X  200.0f  
#define STEPS_PER_MM_Y  200.0f
#define STEPS_PER_MM_Z  400.0f

typedef struct {
    GPIO_TypeDef* port;  
    uint16_t      pin;   
} Plotter_Pin_t;

typedef struct {
    Plotter_Pin_t x_step; Plotter_Pin_t x_dir;
    Plotter_Pin_t y_step; Plotter_Pin_t y_dir;
    Plotter_Pin_t z_step; Plotter_Pin_t z_dir;
} Plotter_Hardware_t;

typedef struct {
    float accel_ratio;  
    float jerk_ratio;   
    float phase_ratios[7]; 
} S_Curve_Config_t;

// 绘图任务结构体 (Bresenham + S型加减速)
typedef struct {
    int32_t dx, dy;           
    int32_t err;              
    int32_t current_step;     
    int32_t total_steps;      
    int8_t  dir_x, dir_y;     
    
    uint16_t current_arr;     
    uint16_t start_arr;       
    uint16_t peak_arr;        
    uint32_t accel_steps;     
    
    int32_t jerk_q16;         
    uint32_t phase_steps[7];
    uint32_t phase_boundaries[7];
    uint8_t accel_phase;
    uint32_t current_phase_step;
    
    volatile uint8_t is_busy; 
    volatile uint8_t limit_triggered; // 限位触发报错标志
    
    // 实时物理绝对步数跟踪
    volatile int32_t abs_step_x;
    volatile int32_t abs_step_y;
} Plotter_Job_t;

// Z轴运动状态结构体
typedef struct {
    volatile uint8_t is_busy;      
    int32_t target_steps;          
    int32_t current_step;          
    uint8_t direction;             
    volatile int32_t abs_step_z;   // Z轴绝对步数
} Z_Axis_State_t;

extern Plotter_Job_t g_plotter_job;
extern Z_Axis_State_t z_axis_state;

void Plotter_Init(Plotter_Hardware_t *hw_config);
void Plotter_StartLine(int32_t target_x, int32_t target_y, uint16_t speed_hz);
void Plotter_SetZ(int32_t target_z, uint16_t speed_hz);
void Plotter_Stop(void);
void Plotter_GetCurrentPosition(float *x_mm, float *y_mm, float *z_mm);

#endif /* __STEPPER_MOTOR_H */