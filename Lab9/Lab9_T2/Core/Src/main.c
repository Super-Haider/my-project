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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ── TASK 1: WHO AM I ─────────────────────────────────────────────────────────
//#define LSM303AGR_ACCEL_ADDR     0x33
//#define LSM303AGR_WHO_AM_I_REG   0x0F
//#define LSM303AGR_WHO_AM_I_VAL   0x33
// ─────────────────────────────────────────────────────────────────────────────

// ── TASK 2: ACCELEROMETER ────────────────────────────────────────────────────
#define LSM303AGR_ACCEL_ADDR_READ   0x33
#define LSM303AGR_ACCEL_ADDR_WRITE  0x32
#define LSM303AGR_WHO_AM_I_REG      0x0F
#define LSM303AGR_WHO_AM_I_VAL      0x33

// Control registers
#define CTRL_REG1_A   0x20
#define CTRL_REG4_A   0x23

// Accelerometer output registers (auto-increment with bit7=1)
#define OUT_X_L_A     0xA8   // 0x28 | 0x80 for auto-increment

// Scaling
#define ACC_SCALE     3.9f        // mg per LSB in normal mode
#define RAD_TO_DEG    57.2958f
#define CALIB_SAMPLES 20

// Struct to hold all LSM data
typedef struct {
    int16_t raw_ax, raw_ay, raw_az;
    float ax, ay, az;
    float acc_offset_x, acc_offset_y, acc_offset_z;
    float angle_x, angle_y;
} LSM_Data;
// ─────────────────────────────────────────────────────────────────────────────

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART redirect
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// ── TASK 2 FUNCTIONS ─────────────────────────────────────────────────────────

void Init_LSM(void)
{
    uint8_t val;

    // Wake up + set ODR to 100Hz, enable X Y Z axes
    val = 0x67;
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACCEL_ADDR_WRITE,
                      CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT,
                      &val, 1, HAL_MAX_DELAY);

    // Normal mode, ±2g
    val = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, LSM303AGR_ACCEL_ADDR_WRITE,
                      CTRL_REG4_A, I2C_MEMADD_SIZE_8BIT,
                      &val, 1, HAL_MAX_DELAY);

    HAL_Delay(10);
}

void Read_LSM(LSM_Data *data)
{
    uint8_t buf[6];

    // Read all 6 bytes at once using auto-increment (bit7 set)
    HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ACCEL_ADDR_READ,
                     OUT_X_L_A, I2C_MEMADD_SIZE_8BIT,
                     buf, 6, HAL_MAX_DELAY);

    // Combine high and low bytes
    data->raw_ax = (int16_t)((buf[1] << 8) | buf[0]);
    data->raw_ay = (int16_t)((buf[3] << 8) | buf[2]);
    data->raw_az = (int16_t)((buf[5] << 8) | buf[4]);

    // Right-shift by 6 (10-bit left-justified), scale to g, subtract offset
    data->ax = ((data->raw_ax >> 6) * ACC_SCALE / 1000.0f) - data->acc_offset_x;
    data->ay = ((data->raw_ay >> 6) * ACC_SCALE / 1000.0f) - data->acc_offset_y;
    data->az = ((data->raw_az >> 6) * ACC_SCALE / 1000.0f) - data->acc_offset_z;

    // Tilt angles in degrees
    data->angle_x = atan2f(data->ay, data->az) * RAD_TO_DEG;
    data->angle_y = atan2f(-data->ax,
                    sqrtf(data->ay * data->ay + data->az * data->az)) * RAD_TO_DEG;
}

void Offset_LSM(LSM_Data *data)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buf[6];

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ACCEL_ADDR_READ,
                         OUT_X_L_A, I2C_MEMADD_SIZE_8BIT,
                         buf, 6, HAL_MAX_DELAY);

        int16_t rx = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t ry = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t rz = (int16_t)((buf[5] << 8) | buf[4]);

        // Right-shift by 6 before scaling
        sum_x += (rx >> 6) * ACC_SCALE / 1000.0f;
        sum_y += (ry >> 6) * ACC_SCALE / 1000.0f;
        sum_z += (rz >> 6) * ACC_SCALE / 1000.0f;

        HAL_Delay(10);
    }

    data->acc_offset_x = sum_x / CALIB_SAMPLES;
    data->acc_offset_y = sum_y / CALIB_SAMPLES;
    // Z: subtract 1g since gravity acts on Z when board is flat
    data->acc_offset_z = (sum_z / CALIB_SAMPLES) - 1.0f;
}
void Print_LSM(LSM_Data *data)
{
    char msg[100];

    // Multiply by 1000 to keep 3 decimal places, cast to int
    int ax     = (int)(data->ax     * 1000);
    int ay     = (int)(data->ay     * 1000);
    int az     = (int)(data->az     * 1000);
    int angleX = (int)(data->angle_x * 100);
    int angleY = (int)(data->angle_y * 100);

    snprintf(msg, sizeof(msg),
             "Ax:%d.%03d,Ay:%d.%03d,Az:%d.%03d,AngleX:%d.%02d,AngleY:%d.%02d\r\n",
             ax/1000, ax%1000 < 0 ? -(ax%1000) : ax%1000,
             ay/1000, ay%1000 < 0 ? -(ay%1000) : ay%1000,
             az/1000, az%1000 < 0 ? -(az%1000) : az%1000,
             angleX/100, angleX%100 < 0 ? -(angleX%100) : angleX%100,
             angleY/100, angleY%100 < 0 ? -(angleY%100) : angleY%100);

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// ─────────────────────────────────────────────────────────────────────────────

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(100);

  // ── TASK 1: Read WHO AM I register ───────────────────────────────────────
  /*
  uint8_t who_am_i = 0;
  char msg[64];

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      &hi2c1,
      LSM303AGR_ACCEL_ADDR,
      LSM303AGR_WHO_AM_I_REG,
      I2C_MEMADD_SIZE_8BIT,
      &who_am_i,
      1,
      HAL_MAX_DELAY
  );

  if (status == HAL_OK)
  {
      snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X\r\n", who_am_i);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

      if (who_am_i == LSM303AGR_WHO_AM_I_VAL)
          snprintf(msg, sizeof(msg), "SUCCESS: LSM303AGR detected!\r\n");
      else
          snprintf(msg, sizeof(msg), "WARNING: Unexpected WHO_AM_I value!\r\n");
  }
  else
  {
      snprintf(msg, sizeof(msg), "ERROR: I2C communication failed! Status = %d\r\n", status);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  */
  // ─────────────────────────────────────────────────────────────────────────

  // ── TASK 2: Initialize and calibrate accelerometer ───────────────────────
  LSM_Data lsm = {0};

  Init_LSM();

  char cal_msg[] = "Calibrating... keep board flat!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)cal_msg, strlen(cal_msg), HAL_MAX_DELAY);

  Offset_LSM(&lsm);

  char done_msg[] = "Calibration done!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)done_msg, strlen(done_msg), HAL_MAX_DELAY);
  // ─────────────────────────────────────────────────────────────────────────

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // ── TASK 1: Print WHO AM I in loop ─────────────────────────────────────
    /*
    uint8_t who_am_i = 0;
    char msg[64];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        LSM303AGR_ACCEL_ADDR,
        LSM303AGR_WHO_AM_I_REG,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i,
        1,
        HAL_MAX_DELAY
    );

    if (status == HAL_OK)
    {
        snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X\r\n", who_am_i);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        if (who_am_i == LSM303AGR_WHO_AM_I_VAL)
            snprintf(msg, sizeof(msg), "SUCCESS: LSM303AGR detected!\r\n");
        else
            snprintf(msg, sizeof(msg), "WARNING: Unexpected WHO_AM_I value!\r\n");
    }
    else
    {
        snprintf(msg, sizeof(msg), "ERROR: I2C failed! Status = %d\r\n", status);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_Delay(1000);
    */
    // ───────────────────────────────────────────────────────────────────────

    // ── TASK 2: Read and print accelerometer data ──────────────────────────
// In while(1), only calibrate when user button is pressed
if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
{
    char cal_msg[] = "Calibrating...\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)cal_msg, strlen(cal_msg), HAL_MAX_DELAY);
    Offset_LSM(&lsm);
    char done_msg[] = "Done!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)done_msg, strlen(done_msg), HAL_MAX_DELAY);
}
    Read_LSM(&lsm);
    Print_LSM(&lsm);
    HAL_Delay(100);
    // ───────────────────────────────────────────────────────────────────────

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MISOA7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */