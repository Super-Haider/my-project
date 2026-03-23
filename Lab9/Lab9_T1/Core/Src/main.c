/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 9 Task 1 — I2C Communication with LSM303AGR
  *                   Read WHO_AM_I_A register (0x0F) from the on-board
  *                   LSM303AGR accelerometer and verify the device ID.
  *                   Expected value : 0x33
  *                   UART output    : USART1 @ 115200 baud
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* ── LSM303AGR I2C Addresses ────────────────────────────────────────────────
 *  STM32 HAL expects the 8-bit address (7-bit address already left-shifted).
 *  Accelerometer:
 *    Write → 0x32   Read → 0x33
 *  Use 0x33 with HAL_I2C_Mem_Read
 *  Use 0x32 with HAL_I2C_Mem_Write
 * ─────────────────────────────────────────────────────────────────────────── */
#define LSM303AGR_ACCEL_READ    0x33   /* 8-bit read  address */
#define LSM303AGR_ACCEL_WRITE   0x32   /* 8-bit write address */

/* ── LSM303AGR Registers ────────────────────────────────────────────────────*/
#define WHO_AM_I_A     0x0F   /* Identification register — expected: 0x33 */
#define CTRL_REG1_A    0x20   /* Power on + enable axes                   */
#define LSM303AGR_ID   0x33   /* Expected WHO_AM_I value                  */
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart1;
PCD_HandleTypeDef  hpcd_USB_FS;

/* USER CODE BEGIN PV */
static uint8_t who_am_i = 0;
static char    uart_buf[80];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
static void UART_Print(const char *msg);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void UART_Print(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_USB_PCD_Init();

    /* USER CODE BEGIN 2 */

    /* Small delay to let UART stabilise before first print */
    HAL_Delay(100);

    UART_Print("\r\n=== Lab 9 Task 1: LSM303AGR WHO_AM_I Test ===\r\n");

    /* ── Step 1: Wake up LSM303AGR accelerometer ───────────────────────────
     *  Write 0x67 to CTRL_REG1_A (0x20):
     *    ODR = 200 Hz, normal mode, X/Y/Z axes enabled
     *  Without this the sensor may not respond correctly.
     * ───────────────────────────────────────────────────────────────────── */
    uint8_t ctrl = 0x67;
    HAL_StatusTypeDef wake_status = HAL_I2C_Mem_Write(
        &hi2c1,
        LSM303AGR_ACCEL_WRITE,
        CTRL_REG1_A,
        I2C_MEMADD_SIZE_8BIT,
        &ctrl,
        1,
        100
    );

    if (wake_status != HAL_OK)
    {
        UART_Print("ERROR: Could not write to LSM303AGR!\r\n");
        UART_Print("Check: PB6=SCL and PB7=SDA are configured in CubeMX.\r\n");
    }
    else
    {
        UART_Print("LSM303AGR wake-up OK.\r\n");
    }

    HAL_Delay(10); /* let sensor settle */

    /* ── Step 2: Read WHO_AM_I_A register (0x0F) ───────────────────────────
     *  HAL_I2C_Mem_Read parameters:
     *    &hi2c1                  — I2C handle
     *    LSM303AGR_ACCEL_READ    — 8-bit read address (0x33)
     *    WHO_AM_I_A (0x0F)       — register to read
     *    I2C_MEMADD_SIZE_8BIT    — 1-byte register address
     *    &who_am_i               — receive buffer
     *    1                       — 1 byte to read
     *    100                     — timeout ms
     * ───────────────────────────────────────────────────────────────────── */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        LSM303AGR_ACCEL_READ,
        WHO_AM_I_A,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i,
        1,
        100
    );

    /* ── Step 3: Print result over UART ────────────────────────────────── */
    int len;

    if (status != HAL_OK)
    {
        len = snprintf(uart_buf, sizeof(uart_buf),
                       "I2C read FAILED (HAL status = %d)\r\n"
                       "Check PB6=SCL and PB7=SDA in CubeMX.\r\n",
                       (int)status);
    }
    else if (who_am_i == LSM303AGR_ID)
    {
        len = snprintf(uart_buf, sizeof(uart_buf),
                       "WHO_AM_I = 0x%02X  -- SUCCESS (LSM303AGR detected!)\r\n",
                       who_am_i);
    }
    else
    {
        len = snprintf(uart_buf, sizeof(uart_buf),
                       "WHO_AM_I = 0x%02X  -- UNEXPECTED (expected 0x33)\r\n",
                       who_am_i);
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, (uint16_t)len, HAL_MAX_DELAY);

    /* USER CODE END 2 */

    while (1)
    {
        HAL_Delay(1000);
    }
}

/**
  * @brief System Clock Configuration
  *        HSE bypass 8 MHz + PLL x6 = 48 MHz
  *        (identical to your original skeleton)
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART1
                                       | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection    = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

/**
  * @brief I2C1 Initialization
  *        SCL = PB6 (AF4)   SDA = PB7 (AF4)
  *        LSM303AGR is on-board — no external wiring needed.
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00201D2B;   /* ~100 kHz with HSI clock */
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

/**
  * @brief SPI1 Initialization (unchanged from skeleton)
  */
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_4BIT;
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
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

/**
  * @brief USART1 Initialization
  *        TX = PA9   RX = PA10   Baud = 115200, 8N1
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance            = USART1;
    huart1.Init.BaudRate       = 115200;
    huart1.Init.WordLength     = UART_WORDLENGTH_8B;
    huart1.Init.StopBits       = UART_STOPBITS_1;
    huart1.Init.Parity         = UART_PARITY_NONE;
    huart1.Init.Mode           = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling   = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/**
  * @brief USB Initialization (unchanged from skeleton)
  */
static void MX_USB_PCD_Init(void)
{
    hpcd_USB_FS.Instance                     = USB;
    hpcd_USB_FS.Init.dev_endpoints           = 8;
    hpcd_USB_FS.Init.speed                   = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.low_power_enable        = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization (unchanged from skeleton)
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                           | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
                           | LD6_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin
                         | MEMS_INT1_Pin | MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                          | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif