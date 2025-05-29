/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : HTS1510 → dual-PWM demo (10 kHz) with watchdog & reset log
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * 
  * APPLICATION OVERVIEW:
  * - Two synchronous 10 kHz PWM outputs for 4-20mA current loop control
  *   • TIM1_CH1 → PA5: Temperature (-60°C to +50°C / -76°F to +122°F)
  *   • TIM3_CH1 → PA6: Pressure (0 psi to 175 psi)
  * - HTS1510 I²C sensor on PB6/PB7 (100 kHz)
  * - UART diagnostics on PA0/PA1 (115200 baud)
  * - 4s watchdog timeout with continuous refresh
  * - PWM resolution: 4800 codes → 0.0365 psi / 0.023 °C per LSB
  * 
  * SIGNAL PROCESSING CHAIN:
  * 
  * Pressure Path:
  * 1. Raw ADC (0-32767) → Clamp to 10%-90% range (3277-29491)
  * 2. Scale to 0-1 fraction: (raw-10%)/(80%)
  * 3. Convert to PSI: 0 + fraction × 175  (updated: 0-175 PSI range)
  * 4. Map to PWM duty: fraction × 4799 counts
  * 5. TIM3→PA6→XTR111→4-20mA current loop
  * 
  * Transfer Function Verification:
  * - At 10% (3277): PSI = 0 + ((3277-3277)/26214)×175 = 0.00 PSI
  * - At 50% (16384): PSI = 0 + ((16384-3277)/26214)×175 = 87.50 PSI  
  * - At 90% (29491): PSI = 0 + ((29491-3277)/26214)×175 = 175.00 PSI
  * - Your raw=7031: PSI = 0 + ((7031-3277)/26214)×175 = 25.05 PSI
  * 
  * Temperature Path:
  * 1. Raw ADC (0-32767) → Direct linear conversion
  * 2. Convert to °C: raw × 0.0156 - 242.8
  * 3. Scale to 0-1 fraction: (T+60)/110
  * 4. Map to PWM duty: fraction × 4799 counts
  * 5. TIM1→PA5→XTR111→4-20mA current loop
  * 
  * TIMING:
  * - 10 Hz: Sensor polling & PWM update
  * - 2 Hz: UART status output
  * - 10 kHz: PWM carrier frequency
  * 
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* PWM Configuration for 10 kHz */
#define PWM_PRESCALER           0U          /* 48 MHz/(0+1)=48 MHz timer clock */
#define PWM_PERIOD              4799U       /* 48 MHz/(4799+1)=10 kHz PWM - 4800 duty codes */

/* HTS1510 Sensor Configuration */
#define HTS1510_ADDR            (0x28U << 1)
#define I2C_TIMEOUT_MS          100U

/* HTS1510 ADC Constants (16-bit left-justified: 0-32767) */
#define ADC_FULL_SCALE          32768.0f
#define ADC_RAW_MIN             (0.10f * ADC_FULL_SCALE)  /* 3277 = 10% FS */
#define ADC_RAW_MAX             (0.90f * ADC_FULL_SCALE)  /* 29491 = 90% FS */
#define ADC_RAW_SPAN            (0.80f * ADC_FULL_SCALE)  /* 26214 = 80% span */

/* Pressure range (datasheet-specified) */
#define PRESS_MIN_PSI           0.0f        /* Changed from 1.0f to 0.0f */
#define PRESS_MAX_PSI           175.0f

/* Temperature range and conversion constants */
#define TEMP_MIN_C              -60.0f
#define TEMP_MAX_C              50.0f
#define TEMP_SCALE              0.015581403267973155f  /* °C/LSB */
#define TEMP_BIAS              -242.7982474855808f     /* °C offset */

/* Timing Configuration */
#define STARTUP_STEP_MS         1000U
#define SENSOR_PERIOD_MS        100U       /* 10 Hz sensor polling */
#define LOG_PERIOD_MS           500U        /* 2 Hz UART logging */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static const uint8_t hts_cmd_press[3] = { 0x2E, 0x21, 0x00 };
static const uint8_t hts_cmd_temp[3]  = { 0x2E, 0x02, 0x00 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void print_reset_cause(void);
static uint16_t hts_read_raw_word(const uint8_t cmd[3]);
static float press_raw_to_psi(uint16_t raw);
static float temp_raw_to_C(uint16_t raw);
static float temp_C_to_F(float temp_c);
static void pwm_update(float pressure_psi, float temperature_C);
int _write(int file, char *ptr, int len);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  /* Print reset cause */
  print_reset_cause();
  
  /* Note about integer-only printf */
  printf("Using integer-only printf (no float support needed)\r\n");
  printf("Temperature displayed in Fahrenheit\r\n");
  
  /* Verify transfer function with your raw value */
  printf("Transfer function check: raw=7031 should be ~25.1 PSI\r\n");
  printf("Formula: PSI = 0 + ((raw-3277)/26214) × 175\r\n\r\n");
  
  /* Fix timer prescaler and period for 10 kHz PWM */
  __HAL_TIM_SET_PRESCALER(&htim1, PWM_PRESCALER);
  __HAL_TIM_SET_AUTORELOAD(&htim1, PWM_PERIOD);
  __HAL_TIM_SET_PRESCALER(&htim3, PWM_PRESCALER);
  __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_PERIOD);
  
  /* Enable TIM1 main output (required for advanced timer) */
  __HAL_TIM_MOE_ENABLE(&htim1);
  
  /* Start PWM outputs */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  /* Startup sweep (0→100→0 %) for visual self-test */
  printf("\r\nStarting 9s PWM sweep (0-25-50-75-100-75-50-25-0%%)...\r\n");
  static const uint8_t startup_duty_pct[] = { 0, 25, 50, 75, 100, 75, 50, 25, 0 };
  for (size_t i = 0; i < sizeof(startup_duty_pct); ++i)
  {
    uint32_t ccr = (startup_duty_pct[i] * PWM_PERIOD) / 100U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(STARTUP_STEP_MS);
  }
  printf("Sweep complete - entering closed-loop sensor mode\r\n");
  printf("Sensor data: 10Hz update, UART log: 2Hz\r\n\r\n");
  
  /* Scheduler variables */
  uint32_t tick_next_sensor = HAL_GetTick();
  uint32_t tick_next_log = HAL_GetTick();
  uint16_t raw_press = 0, raw_temp = 0;
  float press_psi = 0.0f, temp_C = 0.0f, temp_F = 0.0f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t tick_now = HAL_GetTick();
    
    /* 100 ms (10 Hz): Complete sensor → PWM signal chain */
    if ((int32_t)(tick_now - tick_next_sensor) >= 0)
    {
      /* Stage 1: Read raw ADC values from HTS1510 via I2C */
      raw_press = hts_read_raw_word(hts_cmd_press);
      raw_temp = hts_read_raw_word(hts_cmd_temp);
      
      /* Stage 2: Convert to engineering units */
      press_psi = press_raw_to_psi(raw_press);    /* Clamped to 10-90% range */
      temp_C = temp_raw_to_C(raw_temp);           /* Direct linear conversion */
      temp_F = temp_C_to_F(temp_C);               /* Convert to Fahrenheit */
      
      /* Stage 3: Update PWM outputs (0-100% duty → 4-20mA via XTR111) */
      pwm_update(press_psi, temp_C);
      
      /* Keep watchdog alive */
      HAL_IWDG_Refresh(&hiwdg);
      tick_next_sensor += SENSOR_PERIOD_MS;
    }
    
    /* 500 ms (2 Hz): UART diagnostics for human monitoring */
    if ((int32_t)(tick_now - tick_next_log) >= 0)
    {
      /* Integer-only version to avoid printf float requirement */
      int press_int = (int)press_psi;
      int press_dec = (int)((press_psi - press_int) * 100);
      int temp_int = (int)temp_F;
      /* Use fabsf to handle decimal part correctly for negative temps */
      int temp_dec = (int)(fabsf((temp_F - temp_int)) * 10);
      
      /* Handle negative temperatures correctly */
      if (temp_F >= 0) {
        printf("P=%3d.%02d psi (raw=%5u) | T=+%3d.%01d F (raw=%5u)\r\n",
               press_int, press_dec, raw_press, temp_int, temp_dec, raw_temp);
      } else {
        printf("P=%3d.%02d psi (raw=%5u) | T=%4d.%01d F (raw=%5u)\r\n",
               press_int, press_dec, raw_press, temp_int, temp_dec, raw_temp);
      }
      
      tick_next_log += LOG_PERIOD_MS;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00805C87;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 999;
  hiwdg.Init.Reload = 999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void print_reset_cause(void)
{
  printf("\r\nReset cause:");
  bool first = true;
#ifdef RCC_FLAG_IWDGRST
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) { printf(" IWDG"); first = false; }
#endif
#ifdef RCC_FLAG_PORRST
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))  { printf(first?" POR":" |POR"); first=false; }
#endif
#ifdef RCC_FLAG_BORRST
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  { printf(first?" BOR":" |BOR"); first=false; }
#endif
#ifdef RCC_FLAG_PINRST
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  { printf(first?" NRST":" |NRST"); first=false; }
#endif
#ifdef RCC_FLAG_SFTRST
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  { printf(first?" SW":" |SW");   first=false; }
#endif
  if (first) printf(" Unknown");
  printf("\r\n");
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

static uint16_t hts_read_raw_word(const uint8_t cmd[3])
{
  if (HAL_I2C_Master_Transmit(&hi2c1, HTS1510_ADDR, (uint8_t*)cmd, 3, I2C_TIMEOUT_MS) != HAL_OK)
    return 0;
  HAL_Delay(10);  /* ~5 ms conversion time */
  uint8_t rx[3] = {0};
  if (HAL_I2C_Master_Receive(&hi2c1, HTS1510_ADDR, rx, 3, I2C_TIMEOUT_MS) != HAL_OK)
    return 0;
  return (uint16_t)((rx[1] << 8) | rx[2]);
}

static float press_raw_to_psi(uint16_t raw)
{
  /* HTS1510 Transfer Function per datasheet:
   * Ppsi = Pmin + ((Pcounts - 0.1×Max) / (0.8×Max)) × (Pmax - Pmin)
   * 
   * Where:
   * - Pcounts = raw ADC value (0-32767)
   * - Max = 32768 (15-bit full scale)
   * - 0.1×Max = 3276.8 (10% offset)
   * - 0.8×Max = 26214.4 (80% span)
   * - Pmin = 0 PSI, Pmax = 175 PSI
   */
  
  /* Stage 1: Clamp raw ADC to guaranteed linear region (10%-90% FS) */
  float raw_float = (float)raw;
  if (raw_float < ADC_RAW_MIN) raw_float = ADC_RAW_MIN;  /* 3276.8 */
  if (raw_float > ADC_RAW_MAX) raw_float = ADC_RAW_MAX;  /* 29491.2 */
  
  /* Stage 2: Apply transfer function */
  float pressure_fraction = (raw_float - ADC_RAW_MIN) / ADC_RAW_SPAN;
  
  /* Stage 3: Scale to PSI range */
  return PRESS_MIN_PSI + pressure_fraction * (PRESS_MAX_PSI - PRESS_MIN_PSI);
}

static float temp_raw_to_C(uint16_t raw)
{
  /* Direct linear conversion: no clamping needed for temperature */
  return raw * TEMP_SCALE + TEMP_BIAS;
}

static float temp_C_to_F(float temp_c)
{
  /* Convert Celsius to Fahrenheit */
  return (temp_c * 9.0f / 5.0f) + 32.0f;
}

static void pwm_update(float pressure_psi, float temperature_C)
{
  /* Pressure → TIM3/PA6 (for 4-20mA via XTR111) */
  float frac_p = (pressure_psi - PRESS_MIN_PSI) / (PRESS_MAX_PSI - PRESS_MIN_PSI);
  if (frac_p < 0.0f) frac_p = 0.0f;
  if (frac_p > 1.0f) frac_p = 1.0f;
  uint32_t ccr_p = (uint32_t)(frac_p * (float)PWM_PERIOD);

  /* Temperature → TIM1/PA5 (for 4-20mA via XTR111) */
  float frac_t = (temperature_C - TEMP_MIN_C) / (TEMP_MAX_C - TEMP_MIN_C);
  if (frac_t < 0.0f) frac_t = 0.0f;
  if (frac_t > 1.0f) frac_t = 1.0f;
  uint32_t ccr_t = (uint32_t)(frac_t * (float)PWM_PERIOD);

  /* Write to timer compare registers */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr_p);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_t);
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  if (huart1.Instance == NULL) return 0;   /* UART not ready yet */
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

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

#ifdef  USE_FULL_ASSERT
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