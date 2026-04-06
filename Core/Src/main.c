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

// UART接收位置跟踪
static uint32_t last_rx_pos = 0;

// 当前坐标（毫米）
static float cur_x_mm = 0.0f, cur_y_mm = 0.0f, cur_z_mm = 0.0f;

#define STEPS_PER_MM_X  200.0f  // 根据你的实际硬件修改
#define STEPS_PER_MM_Y  200.0f
#define STEPS_PER_MM_Z  400.0f

#define PLOTTER_DEFAULT_SPEED_HZ  5000  // 默认绘图速度 (单位: Hz, 范围 400-12000)

#ifndef PI
#define PI 3.1415926535f
#endif

// 环形指令队列定义
#define COMMAND_QUEUE_SIZE 10

typedef struct {
    float target_x_mm;  // X轴目标位置（毫米）
    float target_y_mm;  // Y轴目标位置（毫米）
    float target_z_mm;  // Z轴目标位置（毫米）
    float speed_hz;     // 运动速度（Hz）
    uint8_t has_xy;     // 是否有XY运动
    uint8_t has_z;      // 是否有Z运动
} Command_t;

typedef struct {
    Command_t commands[COMMAND_QUEUE_SIZE];
    uint8_t head;       // 队首索引（下一个要读取的位置）
    uint8_t tail;       // 队尾索引（下一个要写入的位置）
    uint8_t count;      // 队列中元素数量
} CommandQueue_t;

static CommandQueue_t command_queue = {0};

// 预加载指令状态
static Command_t pending_command;            // 预加载的下一条指令
static uint8_t pending_command_valid = 0;    // 预加载指令是否有效

// 速度过渡状态
static uint8_t speed_transition_enabled = 0; // 是否启用速度过渡
static float transition_speed_ratio = 0.3f;  // 过渡速度比例（30%）
static float current_speed_hz = 0;           // 当前运动速度（Hz）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ParseUARTCommand(uint8_t* buffer, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief 队列管理函数：初始化队列
  */
static void CommandQueue_Init(void)
{
    command_queue.head = 0;
    command_queue.tail = 0;
    command_queue.count = 0;
}

/**
  * @brief 队列管理函数：检查队列是否已满
  * @return 1表示队列已满，0表示未满
  */
static uint8_t CommandQueue_IsFull(void)
{
    return (command_queue.count == COMMAND_QUEUE_SIZE);
}

/**
  * @brief 队列管理函数：检查队列是否为空
  * @return 1表示队列为空，0表示非空
  */
static uint8_t CommandQueue_IsEmpty(void)
{
    return (command_queue.count == 0);
}

/**
  * @brief 队列管理函数：入队
  * @param cmd 要入队的指令
  * @return 1表示成功，0表示队列已满
  */
static uint8_t CommandQueue_Enqueue(Command_t *cmd)
{
    if (CommandQueue_IsFull())
        return 0;
    
    command_queue.commands[command_queue.tail] = *cmd;
    command_queue.tail = (command_queue.tail + 1) % COMMAND_QUEUE_SIZE;
    command_queue.count++;
    return 1;
}

/**
  * @brief 队列管理函数：出队
  * @param cmd 出队的指令存储位置
  * @return 1表示成功，0表示队列为空
  */
static uint8_t CommandQueue_Dequeue(Command_t *cmd)
{
    if (CommandQueue_IsEmpty())
        return 0;
    
    *cmd = command_queue.commands[command_queue.head];
    command_queue.head = (command_queue.head + 1) % COMMAND_QUEUE_SIZE;
    command_queue.count--;
    return 1;
}

/**
  * @brief 队列管理函数：查看队首元素（不出队）
  * @param cmd 队首指令存储位置
  * @return 1表示成功，0表示队列为空
  */
static uint8_t CommandQueue_Peek(Command_t *cmd)
{
    if (CommandQueue_IsEmpty())
        return 0;
    
    *cmd = command_queue.commands[command_queue.head];
    return 1;
}

/**
  * @brief 执行队列中的下一条指令
  * @return 1表示成功执行了指令，0表示队列为空或运动忙
  */
static uint8_t ExecuteNextCommand(void)
{
    // 如果当前有运动正在进行，不能执行新指令
    if (g_plotter_job.is_busy || z_axis_state.is_busy)
        return 0;
    
    Command_t cmd;
    if (!CommandQueue_Dequeue(&cmd))
        return 0; // 队列为空
    
    // 执行XY运动（使用相对位移）
    if (cmd.has_xy)
    {
        // 计算相对位移
        float delta_x_mm = cmd.target_x_mm - cur_x_mm;
        float delta_y_mm = cmd.target_y_mm - cur_y_mm;
        
        int32_t pulse_x = (int32_t)(delta_x_mm * STEPS_PER_MM_X);
        int32_t pulse_y = (int32_t)(delta_y_mm * STEPS_PER_MM_Y);
        Plotter_StartLine(pulse_x, pulse_y, (uint16_t)cmd.speed_hz);
        
        // 更新当前坐标
        cur_x_mm = cmd.target_x_mm;
        cur_y_mm = cmd.target_y_mm;
    }
    
    // 执行Z轴运动（使用相对位移）
    if (cmd.has_z)
    {
        // 计算相对位移
        float delta_z_mm = cmd.target_z_mm - cur_z_mm;
        int32_t pulse_z = (int32_t)(delta_z_mm * STEPS_PER_MM_Z);
        Plotter_SetZ(pulse_z, (uint16_t)cmd.speed_hz);
        
        // 更新当前坐标
        cur_z_mm = cmd.target_z_mm;
    }
    
    return 1;
}

/**
  * @brief 将ARR值转换为速度（Hz）
  * @param arr 定时器ARR值
  * @return 速度（Hz）
  */
static float ARR_To_SpeedHz(uint16_t arr)
{
    if (arr == 0) return 0;
    return 1000000.0f / (arr + 1);
}



/**
  * @brief 计算两个向量之间的夹角（度）
  * @param dx1 向量1的X分量
  * @param dy1 向量1的Y分量
  * @param dx2 向量2的X分量
  * @param dy2 向量2的Y分量
  * @return 夹角（度，0-180）
  */
static float CalculateAngleBetweenVectors(float dx1, float dy1, float dx2, float dy2)
{
    // 计算向量点积
    float dot = dx1 * dx2 + dy1 * dy2;
    // 计算向量模长
    float mag1 = sqrtf(dx1 * dx1 + dy1 * dy1);
    float mag2 = sqrtf(dx2 * dx2 + dy2 * dy2);
    
    // 防止除零
    if (mag1 < 1e-6f || mag2 < 1e-6f)
        return 0.0f;
    
    // 计算余弦值并限制在[-1,1]范围内
    float cos_theta = dot / (mag1 * mag2);
    if (cos_theta > 1.0f) cos_theta = 1.0f;
    if (cos_theta < -1.0f) cos_theta = -1.0f;
    
    // 计算夹角（弧度）并转换为度
    float angle_rad = acosf(cos_theta);
    float angle_deg = angle_rad * 180.0f / PI;
    
    return angle_deg;
}

/**
  * @brief 检查当前运动是否即将结束，并预加载下一条指令
  */
static void CheckAndPreloadNextCommand(void)
{
    // 如果当前没有XY运动，则不需要预加载
    if (!g_plotter_job.is_busy)
        return;
    
    // 计算剩余步数
    int32_t remaining_steps = g_plotter_job.total_steps - g_plotter_job.current_step;
    
    // 如果剩余步数小于等于提前加载阈值（例如总步数的10%或固定步数）
    // 这里使用固定阈值20步，可以根据需要调整
    if (remaining_steps <= 20)
    {
        // 如果还没有预加载下一条指令，则尝试预加载
        if (!pending_command_valid && !CommandQueue_IsEmpty())
        {
            CommandQueue_Peek(&pending_command);
            pending_command_valid = 1;
            
            // 计算当前运动方向向量
            // 注意：g_plotter_job.dir_x和dir_y是方向符号（1或-1），但我们需要实际位移向量
            // 使用dx和dy的符号与方向符号一致
            float current_dx = g_plotter_job.dx * g_plotter_job.dir_x;
            float current_dy = g_plotter_job.dy * g_plotter_job.dir_y;
            
            // 计算下一段运动方向向量
            float next_dx = pending_command.target_x_mm;
            float next_dy = pending_command.target_y_mm;
            
            // 计算夹角
            float angle = CalculateAngleBetweenVectors(current_dx, current_dy, next_dx, next_dy);
            
            // 如果夹角小于10度（直线或微小转角），不启用速度过渡
            if (angle < 10.0f)
            {
                speed_transition_enabled = 0;
            }
            else
            {
                speed_transition_enabled = 1;
                
                // 计算当前速度
                current_speed_hz = ARR_To_SpeedHz(g_plotter_job.current_arr);
                
                // 计算过渡速度（当前速度的30%）
                float transition_speed = current_speed_hz * transition_speed_ratio;
                
                // 限制过渡速度在有效范围内
                if (transition_speed < 400) transition_speed = 400;
                if (transition_speed > 12000) transition_speed = 12000;
                
                // 修改预加载指令的速度为过渡速度
                // 注意：这里修改的是pending_command的副本，实际执行时会使用这个速度
                pending_command.speed_hz = transition_speed;
            }
        }
    }
}

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
    uint8_t is_m114 = 0;
    while (token != NULL)
    {
        // 检查是否为M114命令
        if (strcmp(token, "M114") == 0) {
            is_m114 = 1;
        }
        // 改用 atof 解析浮点数
        else if (token[0] == 'X' || token[0] == 'x') {
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
    
    // 如果是M114命令，立即回传当前位置
    if (is_m114) {
        float x_mm, y_mm, z_mm;
        Plotter_GetCurrentPosition(&x_mm, &y_mm, &z_mm);
        char response[64];
        snprintf(response, sizeof(response), "X%.2f Y%.2f Z%.2f\n", x_mm, y_mm, z_mm);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)response, strlen(response));
        return;
    }
    
    // 构建指令结构体
    Command_t command = {
        .target_x_mm = target_x_mm,
        .target_y_mm = target_y_mm,
        .target_z_mm = target_z_mm,
        .speed_hz = speed_hz,
        .has_xy = has_xy,
        .has_z = has_z
    };
    
    // 入队
    if (!CommandQueue_Enqueue(&command))
    {
        // 队列已满，可以发送错误反馈或丢弃指令
        // 暂时简单丢弃
    }
    
    // 如果当前没有运动，立即执行队列中的指令
    if (!g_plotter_job.is_busy && !z_axis_state.is_busy)
    {
        ExecuteNextCommand();
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
    // 检查UART接收标志
    if (uart_rx_flag)
    {
      uart_rx_flag = 0;
      
      // 计算接收到的数据长度
      uint32_t current_rx_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
      
      if (current_rx_pos > last_rx_pos)
      {
        // 计算新的数据区间长度
        uint16_t new_data_len = current_rx_pos - last_rx_pos;
        
        // 创建临时缓冲区并拷贝新数据
        uint8_t temp_buffer[RX_BUFFER_SIZE];
        memcpy(temp_buffer, rx_buffer + last_rx_pos, new_data_len);
        
        // 解析新接收的命令
        ParseUARTCommand(temp_buffer, new_data_len);
        
        // 更新最后接收位置
        last_rx_pos = current_rx_pos;
      }
      else if (current_rx_pos < last_rx_pos)
      {
        // DMA缓冲区已回绕，处理回绕情况
        uint16_t first_part_len = RX_BUFFER_SIZE - last_rx_pos;
        uint16_t second_part_len = current_rx_pos;
        
        // 创建临时缓冲区并拷贝两部分数据
        uint8_t temp_buffer[RX_BUFFER_SIZE];
        memcpy(temp_buffer, rx_buffer + last_rx_pos, first_part_len);
        memcpy(temp_buffer + first_part_len, rx_buffer, second_part_len);
        
        // 解析新接收的命令
        ParseUARTCommand(temp_buffer, first_part_len + second_part_len);
        
        // 更新最后接收位置
        last_rx_pos = current_rx_pos;
      }
    }
    
    // 检查当前运动是否即将结束，预加载下一条指令
    CheckAndPreloadNextCommand();
    
    // 检查运动任务是否完成
    if (g_plotter_job.is_busy == 0 && z_axis_state.is_busy == 0)
    {
      static uint8_t last_busy_state = 1;
      if (last_busy_state == 1)
      {
        // 仅当队列为空且轴完全停止后，才发送OK\n
        if (CommandQueue_IsEmpty())
        {
          static uint8_t ok_msg[] = "OK\n";
          HAL_UART_Transmit_DMA(&huart1, ok_msg, sizeof(ok_msg)-1);
        }
        last_busy_state = 0;
        
        // 如果有预加载的指令，现在执行它
        if (pending_command_valid)
        {
          // 从队列中移除已预加载的指令（因为我们已经Peek过了）
          Command_t dummy;
          CommandQueue_Dequeue(&dummy); // 丢弃队首，即预加载的指令
          
          // 执行预加载的指令（使用相对位移）
          if (pending_command.has_xy)
          {
            // 计算相对位移
            float delta_x_mm = pending_command.target_x_mm - cur_x_mm;
            float delta_y_mm = pending_command.target_y_mm - cur_y_mm;
            
            int32_t pulse_x = (int32_t)(delta_x_mm * STEPS_PER_MM_X);
            int32_t pulse_y = (int32_t)(delta_y_mm * STEPS_PER_MM_Y);
            Plotter_StartLine(pulse_x, pulse_y, (uint16_t)pending_command.speed_hz);
            
            // 更新当前坐标
            cur_x_mm = pending_command.target_x_mm;
            cur_y_mm = pending_command.target_y_mm;
          }
          
          if (pending_command.has_z)
          {
            // 计算相对位移
            float delta_z_mm = pending_command.target_z_mm - cur_z_mm;
            int32_t pulse_z = (int32_t)(delta_z_mm * STEPS_PER_MM_Z);
            Plotter_SetZ(pulse_z, (uint16_t)pending_command.speed_hz);
            
            // 更新当前坐标
            cur_z_mm = pending_command.target_z_mm;
          }
          
          pending_command_valid = 0;
        }
        else
        {
          // 没有预加载的指令，尝试执行队列中的下一条指令
          ExecuteNextCommand();
        }
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