/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 10 Task 1 — Dynamic Memory Allocation
  *                   Demonstrates malloc(), calloc(), and free() on STM32F3.
  *                   Results transmitted over USART2 at 115200 baud.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>   /* malloc, calloc, free */
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define N  10         /* Number of integer elements to allocate */
/* USER CODE END PD */

I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef  hpcd_USB_FS;

/* USER CODE BEGIN PV */
static char uart_buf[128];
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
/* Helper: send a string over UART */
static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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

    HAL_Delay(100);   /* Let UART settle */

    uart_print("=== Lab 10 Task 1: Dynamic Memory Allocation ===\r\n\r\n");

    /* ════════════════════════════════════════════════════════════════════════
     * STEP 1 — malloc()
     *   Allocate an array of N ints. Memory is UNINITIALIZED (garbage values).
     *   I═══════nitialize each element to i * 2.
     * ═════════════════════════════════════════════════════════════════ */
    uart_print("--- malloc() ---\r\n");

    int *arr_malloc = (int *)malloc(N * sizeof(int));

    if (arr_malloc == NULL)
    {
        uart_print("ERROR: malloc() failed — not enough heap space!\r\n");
    }
    else
    {
        uart_print("malloc() succeeded.\r\n");

        /* Initialize: element i = i * 2 */
        for (int i = 0; i < N; i++)
        {
            arr_malloc[i] = i * 2;
        }

        /* Print contents */
        uart_print("malloc array (i*2): ");
        for (int i = 0; i < N; i++)
        {
            int len = snprintf(uart_buf, sizeof(uart_buf),
                               "%d%s", arr_malloc[i], (i < N - 1) ? ", " : "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, HAL_MAX_DELAY);
        }
    }

    /* ════════════════════════════════════════════════════════════════════════
     * STEP 2 — calloc()
     *   Allocate an array of N ints. Memory IS zero-initialized by calloc().
     *   Confirm all elements are zero, then assign i + 1 to each.
     * ════════════════════════════════════════════════════════════════════════ */
    uart_print("\r\n--- calloc() ---\r\n");

    int *arr_calloc = (int *)calloc(N, sizeof(int));

    if (arr_calloc == NULL)
    {
        uart_print("ERROR: calloc() failed — not enough heap space!\r\n");
    }
    else
    {
        uart_print("calloc() succeeded.\r\n");

        /* Confirm all elements are zero before assignment */
        uart_print("calloc array before init (should all be 0): ");
        for (int i = 0; i < N; i++)
        {
            int len = snprintf(uart_buf, sizeof(uart_buf),
                               "%d%s", arr_calloc[i], (i < N - 1) ? ", " : "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, HAL_MAX_DELAY);
        }

        /* Assign: element i = i + 1 */
        for (int i = 0; i < N; i++)
        {
            arr_calloc[i] = i + 1;
        }

        /* Print contents after assignment */
        uart_print("calloc array after init (i+1): ");
        for (int i = 0; i < N; i++)
        {
            int len = snprintf(uart_buf, sizeof(uart_buf),
                               "%d%s", arr_calloc[i], (i < N - 1) ? ", " : "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, HAL_MAX_DELAY);
        }
    }

    /* ════════════════════════════════════════════════════════════════════════
     * STEP 3 — free() both arrays
     *   Release heap memory and set pointers to NULL to prevent dangling refs.
     * ════════════════════════════════════════════════════════════════════════ */
    uart_print("\r\n--- free() ---\r\n");

    free(arr_malloc);
    arr_malloc = NULL;
    uart_print("arr_malloc freed and set to NULL.\r\n");

    free(arr_calloc);
    arr_calloc = NULL;
    uart_print("arr_calloc freed and set to NULL.\r\n");

    uart_print("\r\nMemory successfully freed. Pointers cleared.\r\n");
    uart_print("=== Task 1 Complete ===\r\n");

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* Task 1 complete — nothing to do in the loop */
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
    hi2c1.Init.Timing           = 0x2000090E;
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