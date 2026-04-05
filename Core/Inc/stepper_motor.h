#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "main.h"
#include <stdint.h>

// 定义单个引脚的结构
typedef struct {
    GPIO_TypeDef* port;  // GPIO 端口 (如 GPIOB)
    uint16_t      pin;   // GPIO 引脚 (如 GPIO_PIN_4)
} Plotter_Pin_t;

// 定义写字机硬件配置总表
typedef struct {
    Plotter_Pin_t x_step;
    Plotter_Pin_t x_dir;
    Plotter_Pin_t y_step;
    Plotter_Pin_t y_dir;
    Plotter_Pin_t z_step;
    Plotter_Pin_t z_dir;
} Plotter_Hardware_t;

// 绘图任务结构体 (Bresenham算法 + T型加减速)
typedef struct {
    // Bresenham算法参数
    int32_t dx, dy;           // X,Y轴差值（绝对值）
    int32_t err;              // 误差累积项
    int32_t current_step;     // 当前已走步数
    int32_t total_steps;      // 总步数（取dx,dy中较大者）
    int8_t  dir_x, dir_y;     // 方向：1为正，-1为负
    
    // T型加减速参数
    uint16_t current_arr;     // 当前ARR值（频率 = 1MHz / (ARR+1)）
    uint16_t start_arr;       // 起始ARR值（对应最低频率）
    uint16_t peak_arr;        // 峰值ARR值（对应最高频率）
    uint32_t accel_steps;     // 加速段步数
    uint16_t arr_step;        // 每步ARR变化量
    
    // 状态标志
    volatile uint8_t is_busy; // 任务忙标志
} Plotter_Job_t;

// 旧版步进电机结构体（保持兼容性）
typedef struct {
    // --- 硬件绑定层 ---
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;

    // --- 软件状态层 ---
    uint8_t  isRunning;          // 0:停止，1:运行
    uint8_t  current_dir;        // 当前物理运行方向 (1正, 0反)
    uint8_t  state;              // 状态机：1-正常加减速, 2-准备换向中
    uint8_t  pending_dir;        // 换向后的新方向
    uint16_t pending_target_arr; // 换向后的最终目标速度
    
    uint16_t current_arr;        // 当前速度对应的 ARR
    uint16_t target_arr;         // 当前阶段目标 ARR
    uint16_t step_change;        // 每次(2ms)改变的 ARR 步长
    uint32_t last_tick;          // 任务时间戳
} StepperMotor_t;

// 全局变量声明
extern Plotter_Job_t g_plotter_job;

// Z轴运动状态结构体
typedef struct {
    volatile uint8_t is_busy;      // Z轴运动忙标志
    int32_t target_steps;          // 目标步数（绝对值）
    int32_t current_step;          // 当前已走步数
    uint8_t direction;             // 方向：1=正方向，0=负方向
} Z_Axis_State_t;

extern Z_Axis_State_t z_axis_state;

// 新版写字机接口函数
void Plotter_Init(Plotter_Hardware_t *hw_config);
void Plotter_StartLine(int32_t target_x, int32_t target_y, uint16_t speed_hz);
void Plotter_SetZ(int32_t target_z, uint16_t speed_hz);
void Plotter_Stop(void);

// 旧版兼容函数（可选保留）
void Stepper_Init(StepperMotor_t *motor, TIM_HandleTypeDef *htim, uint32_t ch, GPIO_TypeDef *port, uint16_t pin);
void Stepper_Start_Soft_Freq(StepperMotor_t *motor, uint8_t dir, float start_freq, float target_freq, uint32_t accel_time_ms);
void Stepper_Stop(StepperMotor_t *motor);
void Stepper_Stop_Soft(StepperMotor_t *motor, uint32_t decel_time_ms);
void Stepper_Run_Task(StepperMotor_t *motor);
void Stepper_Reverse_Soft(StepperMotor_t *motor, uint32_t reverse_decel_time_ms);
void Draw_Line_Angle(StepperMotor_t *motor_x, StepperMotor_t *motor_y, float angle_deg, float total_speed_hz, uint32_t accel_time_ms);

#endif /* __STEPPER_MOTOR_H */
