/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 8 Task 2 - Read Temperature from I3G4250D Gyroscope
  *                   via SPI in Interrupt Mode, transmit over UART
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Gyroscope Register Addresses */
#define CTRL_REG1           0x20    /* Control register 1 */
#define CTRL_REG1_VAL       0x0F    /* PD=1 (power on), Zen=1, Yen=1, Xen=1 */

#define OUT_TEMP_ADDR       0x26    /* Temperature output register */

/* SPI Command Bits */
#define SPI_READ_FLAG       0x80    /* Bit7 = 1 => read operation */

/* Chip Select Pin - PE3 on STM32F3 Discovery */
#define GYRO_CS_PIN         GPIO_PIN_3
#define GYRO_CS_PORT        GPIOE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* SPI state machine */
typedef enum {
    SPI_STATE_IDLE = 0,
    SPI_STATE_TX_DONE,
    SPI_STATE_RX_DONE
} SPI_State_t;

volatile SPI_State_t spiState = SPI_STATE_IDLE;

/* SPI Buffers */
uint8_t tx_buf[1];
uint8_t rx_buf[1];

/* Temperature value (signed 8-bit per datasheet) */
volatile int8_t temperature = 0;

/* UART transmit buffer */
char uart_buf[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
void gyro_init(void);
void gyro_read_temp_start(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Initialize the I3G4250D gyroscope.
  *         Writes to CTRL_REG1 to power on the sensor and enable all axes.
  *         Must be called once in main() before the while(1) loop.
  */
void gyro_init(void)
{
    uint8_t tx[2];
    tx[0] = CTRL_REG1;       /* Register address to write to */
    tx[1] = CTRL_REG1_VAL;   /* Value: PD=1, Zen=1, Yen=1, Xen=1 */

    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    GYRO_CS_HIGH();
}

/**
  * @brief  Start an interrupt-driven SPI read of the temperature register.
  *         Step 1 of 2: Transmit the register address byte (with read bit).
  *         Execution continues in HAL_SPI_TxCpltCallback() once TX is done.
  *
  *         Command byte construction:
  *           SPI_READ_FLAG | OUT_TEMP_ADDR
  *           = 0x80        | 0x26
  *           = 0xA6
  *           = 1010 0110   (bit7=1 => read, bits[5:0]=0x26 => OUT_TEMP)
  */
void gyro_read_temp_start(void)
{
    tx_buf[0] = SPI_READ_FLAG | OUT_TEMP_ADDR;  /* 0xA6 */
    spiState  = SPI_STATE_IDLE;

    GYRO_CS_LOW();
    HAL_SPI_Transmit_IT(&hspi1, tx_buf, 1);     /* Non-blocking TX */
}

/**
  * @brief  SPI Transmit Complete Callback (auto-called by HAL after TX done).
  *         Step 2 of 2: The address byte has been sent; now receive 1 data byte.
  *         CS stays LOW during this call because we are mid-transaction.
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        spiState = SPI_STATE_TX_DONE;
        HAL_SPI_Receive_IT(&hspi1, rx_buf, 1);  /* Non-blocking RX */
    }
}

/**
  * @brief  SPI Receive Complete Callback (auto-called by HAL after RX done).
  *         The temperature byte is now in rx_buf[0].
  *         Pull CS HIGH to end the SPI transaction, then store the value.
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        GYRO_CS_HIGH();

        temperature = (int8_t)rx_buf[0];   /* Cast to signed (datasheet: -1 °C/digit) */
        spiState    = SPI_STATE_RX_DONE;
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

    /* MCU Configuration -------------------------------------------------------*/
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_USB_PCD_Init();

    /* USER CODE BEGIN 2 */

    /* Initialize gyroscope: power on, enable X/Y/Z axes */
    gyro_init();

    /* Small delay to let the sensor stabilize after power-on */
    HAL_Delay(100);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* --- Step 1: Kick off interrupt-based temperature read --- */
        gyro_read_temp_start();

        /* --- Step 2: Wait (spin) until the full SPI transaction completes ---
         *   The HAL callbacks advance spiState:
         *     IDLE -> TX_DONE (in TxCpltCallback)
         *          -> RX_DONE (in RxCpltCallback)
         */
        while (spiState != SPI_STATE_RX_DONE)
        {
            /* Waiting for interrupt callbacks to finish */
        }

        /* --- Step 3: Transmit temperature value over UART ---
         *   Format: signed integer followed by \r\n
         *   Example output line: "27\r\n"
         *   The Python plotting script on the LMS parses this format.
         */
        int len = snprintf(uart_buf, sizeof(uart_buf), "%d\r\n", (int)temperature);
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, HAL_MAX_DELAY);

        /* --- Step 4: Wait ~150 ms before next reading (100-200 ms per lab spec) --- */
        HAL_Delay(150);

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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL         = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2
                                        | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection  = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection    = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection     = RCC_USBCLKSOURCE_PLL;
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
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00201D2B;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
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
  * @brief SPI1 Initialization Function
  *        Mode 3 (CPOL=0, CPHA=1) — matches I3G4250D requirements.
  *        NSS is software-controlled (we drive PE3 manually).
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  *        115200 baud, 8N1 — matches the Python script's serial settings.
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance            = USART2;
    huart2.Init.BaudRate       = 115200;
    huart2.Init.WordLength     = UART_WORDLENGTH_8B;
    huart2.Init.StopBits       = UART_STOPBITS_1;
    huart2.Init.Parity         = UART_PARITY_NONE;
    huart2.Init.Mode           = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USB Initialization Function
  */
static void MX_USB_PCD_Init(void)
{
    hpcd_USB_FS.Instance                = USB;
    hpcd_USB_FS.Init.dev_endpoints      = 8;
    hpcd_USB_FS.Init.speed              = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface         = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.low_power_enable   = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  *        PE3 configured as output for manual Chip Select control.
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Default output level: CS HIGH (deasserted) and LEDs off */
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                           | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
                           | LD6_Pin, GPIO_PIN_RESET);

    /* CS pin starts HIGH — gyroscope not selected */
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    /* Configure DRDY / interrupt input pins */
    GPIO_InitStruct.Pin  = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin
                         | MEMS_INT1_Pin | MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Configure output pins: CS (PE3) and LEDs */
    GPIO_InitStruct.Pin   = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                          | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
                          | LD6_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Configure user button */
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Error handler — disables interrupts and halts.
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */