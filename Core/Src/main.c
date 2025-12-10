/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Collision Detection (Ultrasonic + FreeRTOS)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PIN    GPIO_PIN_14
#define TRIG_PORT   GPIOG
#define ECHO_PIN    GPIO_PIN_13
#define ECHO_PORT   GPIOG

#define LED_PIN     GPIO_PIN_13
#define LED_PORT    GPIOD
#define BUZZ_PIN    GPIO_PIN_12
#define BUZZ_PORT   GPIOD

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 (Sensor) */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask03 (Logic) */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask04 (Alarm) */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 (distance) */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myQueue02 (frequency) */
osMessageQueueId_t myQueue02Handle;
const osMessageQueueAttr_t myQueue02_attributes = {
  .name = "myQueue02"
};

/* USER CODE BEGIN PV */
/* Private variables --------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument); // Sensor
void StartTask03(void *argument); // Logic
void StartTask04(void *argument); // Alarm

/* USER CODE BEGIN PFP */
/* helper prototypes */
static void DWT_Delay_Init(void);
static void delay_us(uint32_t us);
static uint32_t Measure_Echo_Time_us(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Enable DWT for microsecond delays */
  DWT_Delay_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Create the queue(s) */
  myQueue01Handle = osMessageQueueNew (16, sizeof(float), &myQueue01_attributes);    // distance in cm (float)
  myQueue02Handle = osMessageQueueNew (16, sizeof(uint32_t), &myQueue02_attributes);// frequency in Hz (uint32)

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes); // Sensing task
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes); // Logic or the calculation task
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes); // Alarm output as the frequency threshold increases.

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
    /* Should not reach here */
  }
}

/**
  * @brief System Clock Configuration
  *        (This matches cube-generated settings — adjust in CubeMX if needed)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief GPIO Initialization Function
  *        Configures TRIG (PG14), ECHO (PG13), LED (PD13), BUZZ (PD12)
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure PG14 as TRIG output */
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);

  /* Configure PG13 as ECHO input */
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

  /* Configure PD12 (BUZZ) and PD13 (LED) as outputs */
  HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = BUZZ_PIN | LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* ---------------------------
   DWT microsecond delay helpers
   --------------------------- */
static void DWT_Delay_Init(void)
{
  /* Enable DWT and CYCCNT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

}

static void delay_us(uint32_t us)
{
  uint32_t cycles = (SystemCoreClock / 1000000UL) * us;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
}

/* Measure echo pulse width in microseconds (busy-wait with timeout) */
static uint32_t Measure_Echo_Time_us(void)
{
  const uint32_t timeout_us = 30000UL; // 30 ms timeout
  uint32_t t_start = DWT->CYCCNT;
  uint32_t limit = (SystemCoreClock / 1000000UL) * timeout_us;

  /* Wait for rising edge */
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
  {
    if ((DWT->CYCCNT - t_start) > limit)
    	return 0;
  }
  uint32_t t_rise = DWT->CYCCNT;

  /* Wait for falling edge */
  while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
  {
    if ((DWT->CYCCNT - t_start) > limit)
    	return 0;
  }
  uint32_t t_fall = DWT->CYCCNT;

  uint32_t cycles = t_fall - t_rise;
  uint32_t us = cycles / (SystemCoreClock / 1000000UL);
  return us;
}

/* USER CODE END 4 */

/* StartDefaultTask ----------------------------------------------------------*/
void StartDefaultTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
}

/* StartTask02: Sensor Task (Ultrasonic) ------------------------------------*/
void StartTask02(void *argument)
{
  float distance_cm = 0.0f;
  uint32_t echo_us = 0;

  for(;;)
  {
    /* Generate 10 us trigger pulse */
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    /* Measure echo pulse width */
    echo_us = Measure_Echo_Time_us(); // microseconds

    if (echo_us == 0)
    {
      /* timeout or no echo - treat as large distance */
      distance_cm = 999.0f;
    }
    else
    {
      /* distance (cm) = (time_us * 0.0343) / 2 */
      distance_cm = (echo_us * 0.0343f) / 2.0f;
    }

    /* Send distance to logic task */
    osMessageQueuePut(myQueue01Handle, &distance_cm, 0, 0);

    /* Wait before next measurement */
    osDelay(150); // 150 ms
  }
}

/* StartTask03: Logic Task (distance -> frequency) --------------------------*/
void StartTask03(void *argument)
{
  float distance_cm;
  uint32_t frequency;

  for(;;)
  {
    if (osMessageQueueGet(myQueue01Handle, &distance_cm, NULL, osWaitForever) == osOK)
    {
      /* If within threshold -> produce frequency; else 0 */
      if (distance_cm < 15.0f)
      {
        /* Map distance (0..15 cm) inversely to frequency (800..200 Hz)
           closer -> higher freq */
        float t = (15.0f - distance_cm) / 15.0f; // 0..1
        float f = 200.0f + t * (800.0f - 200.0f); // 200..800 Hz
        if (f < 1.0f) f = 0;
        frequency = (uint32_t)f;
      }
      else
      {
        frequency = 0;
      }

      osMessageQueuePut(myQueue02Handle, &frequency, 0, 0);
    }
  }
}

/* StartTask04: Alarm Task (LED + Buzzer) ----------------------------------*/
void StartTask04(void *argument)
{
  uint32_t freq;
  for(;;)
  {
    if (osMessageQueueGet(myQueue02Handle, &freq, NULL, osWaitForever) == osOK)
    {
      if (freq > 0)
      {
        /* clamp freq to avoid division by zero and extreme values */
        if (freq > 5000) freq = 5000;

        /* compute half period in microseconds (50% duty) */
        uint32_t half_period_us = (500000U / (freq > 0 ? freq : 1)); // 500000/freq = (1000ms/2)/freq *1000 -> microsec
        if (half_period_us == 0) half_period_us = 1;

        /* Produce tone for a short burst then yield to scheduler */
        /* We'll produce 20 full cycles then yield */
        for (uint32_t i = 0; i < 20; ++i)
        {
          HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
          HAL_GPIO_TogglePin(BUZZ_PORT, BUZZ_PIN);
          delay_us(half_period_us);
        }
        /* After burst, small pause */
        osDelay(10);
      }
      else
      {
        /* No tone - ensure pins off */
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
        osDelay(100);
      }
    }
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add reporting here */
}
#endif /* USE_FULL_ASSERT */
