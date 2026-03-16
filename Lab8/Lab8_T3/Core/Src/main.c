/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 8 Task 3 — Angular Velocity (X,Y,Z) + Temperature
  *                   via SPI polling. NO floats used anywhere.
  *                   Output: "<temp>, <x_dps>, <y_dps>, <z_dps>\r\n"
  *                   e.g.  "25, 1.23, -0.87, 3.14\r\n"
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define CTRL_REG1      0x20
#define CTRL_REG1_VAL  0x8F   /* PD=1, Zen=1, Yen=1, Xen=1, ODR=400Hz */

#define CTRL_REG4      0x23
#define CTRL_REG4_VAL  0x00   /* FS = ±245 dps */

#define OUT_TEMP       0x26
#define OUT_X_L        0x28

#define SPI_READ       0x80
#define SPI_AUTO_INC   0x40

#define GYRO_CS_PIN    GPIO_PIN_3
#define GYRO_CS_PORT   GPIOE
/* USER CODE END PD */

/* USER CODE BEGIN PM */
#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET)
/* USER CODE END PM */

I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef  hpcd_USB_FS;

/* USER CODE BEGIN PV */
static uint8_t  rx_buf[6];
static int8_t   temp_raw;
static int16_t  x_raw, y_raw, z_raw;
static char     uart_buf[64];
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
static void    gyro_write_reg(uint8_t reg, uint8_t val);
static uint8_t gyro_read_reg(uint8_t reg);
static void    gyro_init(void);
static void    gyro_read_all(void);
static int     write_dps(char *buf, int32_t raw);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Write one register */
static void gyro_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x3F, val };
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    GYRO_CS_HIGH();
}

/* Read one register */
static uint8_t gyro_read_reg(uint8_t reg)
{
    uint8_t cmd = SPI_READ | (reg & 0x3F);
    uint8_t data = 0;
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd,  1, HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, &data, 1, HAL_MAX_DELAY);
    GYRO_CS_HIGH();
    return data;
}

/* Power on sensor */
static void gyro_init(void)
{
    gyro_write_reg(CTRL_REG1, CTRL_REG1_VAL);
    gyro_write_reg(CTRL_REG4, CTRL_REG4_VAL);
}

/* Read temp + XYZ */
static void gyro_read_all(void)
{
    temp_raw = (int8_t)gyro_read_reg(OUT_TEMP);

    uint8_t cmd = SPI_READ | SPI_AUTO_INC | OUT_X_L;  /* 0xE8 */
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd,   1, HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, rx_buf, 6, HAL_MAX_DELAY);
    GYRO_CS_HIGH();

    x_raw = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    y_raw = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    z_raw = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}

/**
  * @brief  Convert a raw gyro value to a "X.XX" dps string WITHOUT using floats.
  *
  *  Sensitivity = 8.75 mdps/LSB
  *  raw * 875 / 100  gives value in units of 0.1 mdps  (i.e. micro-dps × 10)
  *  Dividing by 1000 gives whole dps; remainder/10 gives 2 decimal places.
  *
  *  Examples:
  *    raw =  141  →  141*875/100 = 1233  → "1.23"
  *    raw = -100  → -100*875/100 = -875  → "-0.87"
  *    raw =    0  →   "0.00"
  *
  * @param buf  destination (needs ≥12 bytes)
  * @param raw  int16_t raw sensor reading
  * @retval characters written
  */
static int write_dps(char *buf, int32_t raw)
{
    /* Scale to 0.01 dps units: raw * 875 / 100  (= raw * 8.75) */
    int32_t scaled = (raw * 875L) / 100L;   /* units of 0.01 dps */

    int negative = (scaled < 0);
    if (negative) scaled = -scaled;

    int32_t whole = scaled / 100;           /* integer dps part   */
    int32_t frac  = scaled % 100;           /* two decimal places */

    if (negative)
        return sprintf(buf, "-%ld.%02ld", (long)whole, (long)frac);
    else
        return sprintf(buf,  "%ld.%02ld", (long)whole, (long)frac);
}

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_USB_PCD_Init();

    /* USER CODE BEGIN 2 */
    GYRO_CS_HIGH();          /* Deassert CS */
    gyro_init();
    HAL_Delay(100);
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        gyro_read_all();

        /* Build output string using only integer sprintf (%ld, %d) */
        char xs[12], ys[12], zs[12];
        write_dps(xs, x_raw);
        write_dps(ys, y_raw);
        write_dps(zs, z_raw);

        int len = sprintf(uart_buf, "%d, %s, %s, %s\r\n",
                          (int)temp_raw, xs, ys, zs);

        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, HAL_MAX_DELAY);

        HAL_Delay(100);
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

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

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2
                                       | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection    = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

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
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

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
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

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
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

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

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif