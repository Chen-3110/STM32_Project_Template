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
// UART接收缓冲区
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t uart_rx_flag = 0;

// 绘图机硬件配置
Plotter_Hardware_t plotter_hw;

// 外部定义的绘图任务
extern Plotter_Job_t g_plotter_job;

// 外部定义的Z轴状态
extern Z_Axis_State_t z_axis_state;

// 外部定义的DMA句柄
extern DMA_HandleTypeDef hdma_usart1_rx;

// 运动完成状态跟踪
static uint8_t last_busy_state = 1;

#define STEPS_PER_MM_X  200.0f  // 根据你的实际硬件修改
#define STEPS_PER_MM_Y  200.0f
#define STEPS_PER_MM_Z  400.0f

#define PLOTTER_DEFAULT_SPEED_HZ  5000  // 默认绘图速度 (单位: Hz, 范围 400-12000)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ParseUARTCommand(uint8_t* buffer, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief 解析UART命令，格式为"X100 Y200 Z10\n"
  * @param buffer 接收缓冲区
  * @param len 数据长度
  */
void ParseUARTCommand(uint8_t* buffer, uint16_t len)
{
    if (len == 0 || buffer[len-1] != '\n' || len >= RX_BUFFER_SIZE)
        return;
    
    buffer[len-1] = '\0';
    char* cmd = (char*)buffer;
    
    // 改用 float 接收上位机传来的物理位移（如 X10.5）
    float target_x_mm = 0, target_y_mm = 0, target_z_mm = 0;
    uint8_t has_xy = 0, has_z = 0;
    float speed_hz = PLOTTER_DEFAULT_SPEED_HZ; // 默认速度

    char* token = strtok(cmd, " ");
    while (token != NULL)
    {
        // 改用 atof 解析浮点数
        if (token[0] == 'X' || token[0] == 'x') {
            target_x_mm = atof(token + 1);
            has_xy = 1;
        }
        else if (token[0] == 'Y' || token[0] == 'y') {
            target_y_mm = atof(token + 1);
            has_xy = 1;
        }
        else if (token[0] == 'Z' || token[0] == 'z') {
            target_z_mm = atof(token + 1);
            has_z = 1;
        }
        else if (token[0] == 'F' || token[0] == 'f') {
            // 解析速度参数
            speed_hz = atof(token + 1);
            // 速度范围限制
            if (speed_hz < 400) speed_hz = 400;
            if (speed_hz > 12000) speed_hz = 12000;
        }
        token = strtok(NULL, " ");
    }
    
    // 将物理位移转换为实际脉冲数
    if (has_xy)
    {
        int32_t pulse_x = (int32_t)(target_x_mm * STEPS_PER_MM_X);
        int32_t pulse_y = (int32_t)(target_y_mm * STEPS_PER_MM_Y);
        Plotter_StartLine(pulse_x, pulse_y, speed_hz); // 使用解析的速度参数
    }
    
    if (has_z)
    {
        int32_t pulse_z = (int32_t)(target_z_mm * STEPS_PER_MM_Z);
        Plotter_SetZ(pulse_z, speed_hz); // 使用解析的速度参数
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
    // 检查UART接收标志
    if (uart_rx_flag)
    {
      uart_rx_flag = 0;
      
      // 计算接收到的数据长度
      uint16_t rx_len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
      
      if (rx_len > 0 && rx_len < RX_BUFFER_SIZE)
      {
        // 解析命令
        ParseUARTCommand(rx_buffer, rx_len);
      }
    }
    
    // 检查运动任务是否完成，发送OK反馈
    // 仅当X/Y轴插补和Z轴运动全部真正结束后才发送OK\n
    if (g_plotter_job.is_busy == 0 && z_axis_state.is_busy == 0)
    {
      static uint8_t last_busy_state = 1;
      if (last_busy_state == 1)
      {
        // 运动完成，发送OK\n
        static uint8_t ok_msg[] = "OK\n";
        HAL_UART_Transmit_DMA(&huart1, ok_msg, sizeof(ok_msg)-1);
        last_busy_state = 0;
      }
    }
    else
    {
      last_busy_state = 1;
    }
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
