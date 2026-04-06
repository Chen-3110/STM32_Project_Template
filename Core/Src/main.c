/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "soft_timer.h"
#include "stepper_motor.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t uart_rx_flag = 0;
static uint32_t last_rx_pos = 0; // DMA 循环缓冲区读指针

Plotter_Hardware_t plotter_hw;
extern Plotter_Job_t g_plotter_job;
extern Z_Axis_State_t z_axis_state;
extern DMA_HandleTypeDef hdma_usart1_rx;

#define PLOTTER_DEFAULT_SPEED_HZ  5000 

#ifndef PI
#define PI 3.1415926535f
#endif

// 环形指令队列
#define COMMAND_QUEUE_SIZE 10
typedef struct {
    float target_x_mm;  
    float target_y_mm;  
    float target_z_mm;  
    float speed_hz;     
    uint8_t has_xy;     
    uint8_t has_z;      
} Command_t;

typedef struct {
    Command_t commands[COMMAND_QUEUE_SIZE];
    uint8_t head;       
    uint8_t tail;       
    uint8_t count;      
} CommandQueue_t;

static CommandQueue_t command_queue = {0};

// 解析层：用于跟踪上位机发送的“逻辑绝对坐标”
static float last_queued_x = 0, last_queued_y = 0, last_queued_z = 0;
// 执行层：用于跟踪下位机正在执行的“起始坐标”
static float executed_x = 0, executed_y = 0, executed_z = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ParseUARTCommand(char* cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 队列基础函数
static void CommandQueue_Init(void) { command_queue.head = 0; command_queue.tail = 0; command_queue.count = 0; }
static uint8_t CommandQueue_IsFull(void) { return (command_queue.count == COMMAND_QUEUE_SIZE); }
static uint8_t CommandQueue_IsEmpty(void) { return (command_queue.count == 0); }
static uint8_t CommandQueue_Enqueue(Command_t *cmd) {
    if (CommandQueue_IsFull()) return 0;
    command_queue.commands[command_queue.tail] = *cmd;
    command_queue.tail = (command_queue.tail + 1) % COMMAND_QUEUE_SIZE;
    command_queue.count++;
    return 1;
}
static uint8_t CommandQueue_Dequeue(Command_t *cmd) {
    if (CommandQueue_IsEmpty()) return 0;
    *cmd = command_queue.commands[command_queue.head];
    command_queue.head = (command_queue.head + 1) % COMMAND_QUEUE_SIZE;
    command_queue.count--;
    return 1;
}

// 安全发送函数：防止 DMA 碰撞死锁
static void Send_UART_Msg(const char* msg) {
    while (huart1.gState != HAL_UART_STATE_READY); // 等待上一个传输完成
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
}

// 执行队列中的指令
static void ExecuteNextCommand(void) {
    if (g_plotter_job.is_busy || z_axis_state.is_busy) return;
    
    Command_t cmd;
    if (!CommandQueue_Dequeue(&cmd)) return;
    
    // 计算相对位移（脉冲数）
    if (cmd.has_xy) {
        int32_t pulse_x = (int32_t)((cmd.target_x_mm - executed_x) * STEPS_PER_MM_X);
        int32_t pulse_y = (int32_t)((cmd.target_y_mm - executed_y) * STEPS_PER_MM_Y);
        Plotter_StartLine(pulse_x, pulse_y, (uint16_t)cmd.speed_hz);
        executed_x = cmd.target_x_mm;
        executed_y = cmd.target_y_mm;
    }
    if (cmd.has_z) {
        int32_t pulse_z = (int32_t)((cmd.target_z_mm - executed_z) * STEPS_PER_MM_Z);
        Plotter_SetZ(pulse_z, (uint16_t)cmd.speed_hz);
        executed_z = cmd.target_z_mm;
    }
}

// 解析指令
void ParseUARTCommand(char* cmd) {
    float target_x_mm = last_queued_x; 
    float target_y_mm = last_queued_y; 
    float target_z_mm = last_queued_z;
    uint8_t has_xy = 0, has_z = 0;
    float speed_hz = PLOTTER_DEFAULT_SPEED_HZ; 

    // 处理实时位置查询指令
    if (strstr(cmd, "M114") != NULL) {
        float rx, ry, rz;
        Plotter_GetCurrentPosition(&rx, &ry, &rz);
        static char response[64];
        snprintf(response, sizeof(response), "X%.2f Y%.2f Z%.2f\n", rx, ry, rz);
        Send_UART_Msg(response);
        return;
    }

    char* token = strtok(cmd, " ");
    while (token != NULL) {
        if (token[0] == 'X' || token[0] == 'x') { target_x_mm = atof(token + 1); has_xy = 1; }
        else if (token[0] == 'Y' || token[0] == 'y') { target_y_mm = atof(token + 1); has_xy = 1; }
        else if (token[0] == 'Z' || token[0] == 'z') { target_z_mm = atof(token + 1); has_z = 1; }
        else if (token[0] == 'F' || token[0] == 'f') { speed_hz = atof(token + 1); }
        token = strtok(NULL, " ");
    }
    
    // 如果包含有效位移，构建并压入队列
    if (has_xy || has_z) {
        Command_t command = {target_x_mm, target_y_mm, target_z_mm, speed_hz, has_xy, has_z};
        if (CommandQueue_Enqueue(&command)) {
            // 解析并排队成功，更新逻辑坐标，并立即回复OK
            if(has_xy){ last_queued_x = target_x_mm; last_queued_y = target_y_mm; }
            if(has_z) { last_queued_z = target_z_mm; }
            Send_UART_Msg("OK\n"); 
        } else {
            Send_UART_Msg("Error: Queue Full\n");
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 初始化DWT微秒级延时
  DWT_Init();
  
  // 配置绘图机硬件引脚映射（根据PROJECT_CONTEXT.md中的硬件映射）
  plotter_hw.x_step.port = GPIOB;
  plotter_hw.x_step.pin = GPIO_PIN_10;
  plotter_hw.x_dir.port = GPIOB;
  plotter_hw.x_dir.pin = GPIO_PIN_11;
  
  plotter_hw.y_step.port = GPIOB;
  plotter_hw.y_step.pin = GPIO_PIN_12;
  plotter_hw.y_dir.port = GPIOB;
  plotter_hw.y_dir.pin = GPIO_PIN_13;
  
  plotter_hw.z_step.port = GPIOB;
  plotter_hw.z_step.pin = GPIO_PIN_0;  // TIM3_CH3 (PB0) 用于Z轴脉冲
  plotter_hw.z_dir.port = GPIOB;
  plotter_hw.z_dir.pin = GPIO_PIN_1;
  
  // 初始化绘图机硬件
  Plotter_Init(&plotter_hw);
  
  // 初始化指令队列
  CommandQueue_Init();
  
  // 启动UART1 DMA接收（循环模式）并启用IDLE中断
  HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  
  // 使能电机（PB2低电平有效）
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  
  // 启动TIM6作为运动心脏（1MHz，12kHz IRQ）
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 1. 处理UART接收：解决断包和循环覆盖问题
    if (uart_rx_flag) {
      uart_rx_flag = 0;
      uint32_t curr_rx_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
      
      static char line_buf[128];
      static uint16_t line_idx = 0;
      
      while (last_rx_pos != curr_rx_pos) {
          char c = (char)rx_buffer[last_rx_pos];
          last_rx_pos = (last_rx_pos + 1) % RX_BUFFER_SIZE;
          
          if (c == '\n' || c == '\r') {
              if (line_idx > 0) {
                  line_buf[line_idx] = '\0';
                  ParseUARTCommand(line_buf); // 解析完整的行
                  line_idx = 0;
              }
          } else if (line_idx < sizeof(line_buf) - 1) {
              line_buf[line_idx++] = c;
          }
      }
    }
    
    // 2. 限位报错处理（将ISR中的耗时操作移出到主循环）
    if (g_plotter_job.limit_triggered) {
        g_plotter_job.limit_triggered = 0;
        Send_UART_Msg("Error: Limit Switch Triggered!\n");
        CommandQueue_Init(); // 清空队列，防止电机继续撞击

        // 【修复】重新同步底层的实际物理位置，防止下一次绘图坐标错乱
        Plotter_GetCurrentPosition(&executed_x, &executed_y, &executed_z);
        last_queued_x = executed_x;
        last_queued_y = executed_y;
        last_queued_z = executed_z;
    }
    
    // 3. 自动执行下一条指令
    ExecuteNextCommand();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */