#include "soft_timer.h"

/*============================================================
 *  DWT 寄存器定义与初始化
 *============================================================*/
#define DWT_CTRL_REG   (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT_REG (*(volatile uint32_t *)0xE0001004)
#define DEMCR_REG      (*(volatile uint32_t *)0xE000EDFC)

void DWT_Init(void)
{
    DEMCR_REG      |= (1 << 24);   // 启用DWT模块
    DWT_CYCCNT_REG  = 0;            // 清零计数器
    DWT_CTRL_REG   |= (1 << 0);    // 开始计数
}

/*============================================================
 *  毫秒级非阻塞定时器
 *============================================================*/
void Timer_ms_Start(SoftTimer_ms *t, uint32_t ms)
{
    t->start   = HAL_GetTick();
    t->delay   = ms;
    t->running = 1;
}

void Timer_ms_Stop(SoftTimer_ms *t)
{
    t->running = 0;
}

uint8_t Timer_ms_IsRunning(SoftTimer_ms *t)
{
    return t->running;
}

/* 单次：到期返回1并自动停止，之后不再触发 */
uint8_t Timer_ms_IsExpired(SoftTimer_ms *t)
{
    if (!t->running) return 0;
    if ((HAL_GetTick() - t->start) >= t->delay)
    {
        t->running = 0;
        return 1;
    }
    return 0;
}

/* 周期：到期返回1并自动重启，持续周期触发 */
uint8_t Timer_ms_Check(SoftTimer_ms *t)
{
    if (!t->running) return 0;
    if ((HAL_GetTick() - t->start) >= t->delay)
    {
        t->start += t->delay;  // 累加避免误差积累
        return 1;
    }
    return 0;
}

/*============================================================
 *  微秒级非阻塞定时器
 *============================================================*/
void Timer_us_Start(SoftTimer_us *t, uint32_t us)
{
    t->start   = DWT_CYCCNT_REG;
    t->delay   = us * (SystemCoreClock / 1000000);  // 72MHz: 1us = 72 ticks
    t->running = 1;
}

void Timer_us_Stop(SoftTimer_us *t)
{
    t->running = 0;
}

uint8_t Timer_us_IsRunning(SoftTimer_us *t)
{
    return t->running;
}

/* 单次 */
uint8_t Timer_us_IsExpired(SoftTimer_us *t)
{
    if (!t->running) return 0;
    if ((DWT_CYCCNT_REG - t->start) >= t->delay)
    {
        t->running = 0;
        return 1;
    }
    return 0;
}

/* 周期 */
uint8_t Timer_us_Check(SoftTimer_us *t)
{
    if (!t->running) return 0;
    if ((DWT_CYCCNT_REG - t->start) >= t->delay)
    {
        t->start += t->delay;
        return 1;
    }
    return 0;
}

/*============================================================
 *  阻塞延时
 *============================================================*/
void Delay_ms(uint32_t ms)
{
    SoftTimer_ms t;
    Timer_ms_Start(&t, ms);
    while (!Timer_ms_IsExpired(&t));
}

void Delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT_REG;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT_CYCCNT_REG - start) < ticks);
}