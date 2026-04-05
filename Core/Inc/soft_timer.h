#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#include "stm32f1xx_hal.h"

/*============================================================
 *  DWT 初始化（使用微秒定时器前必须调用一次）
 *============================================================*/
void DWT_Init(void);

/*============================================================
 *  毫秒级非阻塞定时器（基于 HAL_GetTick，最大约49天）
 *============================================================*/
typedef struct {
    uint32_t start;
    uint32_t delay;
    uint8_t  running;
} SoftTimer_ms;

void    Timer_ms_Start(SoftTimer_ms *t, uint32_t ms);
void    Timer_ms_Stop(SoftTimer_ms *t);
uint8_t Timer_ms_IsRunning(SoftTimer_ms *t);
uint8_t Timer_ms_IsExpired(SoftTimer_ms *t);   // 单次：到期返回1，自动停止
uint8_t Timer_ms_Check(SoftTimer_ms *t);       // 周期：到期返回1，自动重启

/*============================================================
 *  微秒级非阻塞定时器（基于 DWT，最大约59秒 @72MHz）
 *============================================================*/
typedef struct {
    uint32_t start;
    uint32_t delay;
    uint8_t  running;
} SoftTimer_us;

void    Timer_us_Start(SoftTimer_us *t, uint32_t us);
void    Timer_us_Stop(SoftTimer_us *t);
uint8_t Timer_us_IsRunning(SoftTimer_us *t);
uint8_t Timer_us_IsExpired(SoftTimer_us *t);   // 单次
uint8_t Timer_us_Check(SoftTimer_us *t);       // 周期

/*============================================================
 *  阻塞延时（替代 HAL_Delay，额外提供微秒版）
 *============================================================*/
void Delay_ms(uint32_t ms);
void Delay_us(uint32_t us);

#endif /* SOFT_TIMER_H */
