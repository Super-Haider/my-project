/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 6 Task 4 - Motor Speed Measurement Using Input Capture
  ******************************************************************************
  * @attention
  *
  * Hardware Setup (Keyestudio Balance Car Shield):
  * - D4 (Encoder)  -> PA0 (TIM3_CH1) - Input Capture
  * - D10 (PWM)     -> PWM output (TIM2_CH1) - Right motor
  * - D9 (PWM)      -> PWM output (TIM2_CH2) - Left motor
  * - D12           -> PA5 - Right motor direction 1
  * - D8            -> PA6 - Right motor direction 2
  * - D7            -> PC4 - Left motor direction 1
  * - D6            -> PC5 - Left motor direction 2
  * - 5V            -> 5V - Encoder power
  * - GND           -> GND - Common ground
  * - Vin           -> External 7-12V - Motor power
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Structure to store encoder measurements
typedef struct {
    uint32_t pulse_period;      // Time between pulses in microseconds
    uint32_t pulse_count;       // Number of pulses counted
    float frequency;            // Calculated frequency in Hz
    float rpm;                  // Calculated RPM
    uint8_t valid;              // Flag indicating valid measurement
} Encoder_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== LAB 6 TASK 4: MOTOR SPEED MEASUREMENT USING INPUT CAPTURE =====

// Motor PWM Pins (TIM2)
#define MOTOR_RIGHT_PWM_PORT   GPIOA
#define MOTOR_RIGHT_PWM_PIN    GPIO_PIN_1   // PA1 - TIM2_CH2 (if using CH2)
#define MOTOR_LEFT_PWM_PORT    GPIOA
#define MOTOR_LEFT_PWM_PIN     GPIO_PIN_2   // PA2 - TIM2_CH3 (if using CH3)

// Motor Direction Pins (from shield)
#define MOTOR_RIGHT_DIR1_PORT  GPIOA
#define MOTOR_RIGHT_DIR1_PIN   GPIO_PIN_5   // D12 - Right motor direction 1
#define MOTOR_RIGHT_DIR2_PORT  GPIOA
#define MOTOR_RIGHT_DIR2_PIN   GPIO_PIN_6   // D8 - Right motor direction 2

#define MOTOR_LEFT_DIR1_PORT   GPIOC
#define MOTOR_LEFT_DIR1_PIN    GPIO_PIN_4   // D7 - Left motor direction 1
#define MOTOR_LEFT_DIR2_PORT   GPIOC
#define MOTOR_LEFT_DIR2_PIN    GPIO_PIN_5   // D6 - Left motor direction 2

// Encoder Parameters
#define PULSES_PER_REVOLUTION  330    // PPR - Adjust based on your motor
#define MIN_VALID_PERIOD       100   // Minimum valid period (100µs = 10kHz max)
#define MAX_VALID_PERIOD       100000 // Maximum valid period (100ms = 10Hz min)

// PWM Parameters for Motor Control
#define PWM_MAX_VALUE          4294967295  // 32-bit max for TIM2
#define PWM_TEST_SPEED          2147483647  // 50% duty cycle

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* ============================================================
   TASK 4: Input Capture Variables
   ============================================================ */
volatile uint32_t ic_val1       = 0;   // First captured timer value
volatile uint32_t ic_val2       = 0;   // Second captured timer value
volatile uint8_t  ic_edge_count = 0;   // Counts how many edges have been captured
volatile uint8_t  ic_ready      = 0;   // Flag: 1 = two edges captured, RPM ready to calc

// Encoder data for right motor
static Encoder_Data_t encoder_right = {0};

// UART buffer for printf
static char uart_buffer[256];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */

// ===== MOTOR CONTROL FUNCTIONS =====
void Motor_Init(void);
void Motor_SetSpeed_Right(uint32_t pwm_value, uint8_t direction);
void Motor_SetSpeed_Left(uint32_t pwm_value, uint8_t direction);
void Motor_Stop(void);

// ===== ENCODER FUNCTIONS =====
void Encoder_Init_InputCapture(void);
float Encoder_Calculate_Frequency(uint32_t period_us);
float Encoder_Calculate_RPM(float frequency_hz);

// ===== UART FUNCTIONS =====
void UART_Printf(const char *format, ...);
void UART_SendString(char *str);

// ===== TASK 4 FUNCTIONS =====
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void calculate_and_print_RPM(void);
void Print_Motor_Speed_InputCapture(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ===== UART FUNCTIONS =====

/**
  * @brief Simplified printf using UART
  */
void UART_Printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(uart_buffer, sizeof(uart_buffer), format, args);
    va_end(args);
    
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}

/**
  * @brief Send string over UART
  */
void UART_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

// ===== MOTOR CONTROL FUNCTIONS =====

/**
  * @brief Initialize motor control pins and PWM
  */
void Motor_Init(void)
{
    // Initialize direction pins (set all to output low)
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR1_PORT, MOTOR_RIGHT_DIR1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR2_PORT, MOTOR_RIGHT_DIR2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR1_PORT, MOTOR_LEFT_DIR1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR2_PORT, MOTOR_LEFT_DIR2_PIN, GPIO_PIN_RESET);
    
    // Start PWM on motor channels
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Right motor PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Left motor PWM
    
    // Initialize with motor stopped
    Motor_Stop();
    
    UART_Printf("Motor initialized\r\n");
}

/**
  * @brief Set right motor speed and direction
  * @param pwm_value PWM value (0-4294967295)
  * @param direction 0 = forward, 1 = backward
  */
void Motor_SetSpeed_Right(uint32_t pwm_value, uint8_t direction)
{
    // Set direction pins according to Keyestudio shield logic
    // Forward: D8 LOW, D12 HIGH
    // Backward: D8 HIGH, D12 LOW
    if (direction == 0)  // Forward
    {
        HAL_GPIO_WritePin(MOTOR_RIGHT_DIR1_PORT, MOTOR_RIGHT_DIR1_PIN, GPIO_PIN_SET);   // D12 HIGH
        HAL_GPIO_WritePin(MOTOR_RIGHT_DIR2_PORT, MOTOR_RIGHT_DIR2_PIN, GPIO_PIN_RESET); // D8 LOW
    }
    else  // Backward
    {
        HAL_GPIO_WritePin(MOTOR_RIGHT_DIR1_PORT, MOTOR_RIGHT_DIR1_PIN, GPIO_PIN_RESET); // D12 LOW
        HAL_GPIO_WritePin(MOTOR_RIGHT_DIR2_PORT, MOTOR_RIGHT_DIR2_PIN, GPIO_PIN_SET);   // D8 HIGH
    }
    
    // Set PWM duty cycle
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
}

/**
  * @brief Set left motor speed and direction
  * @param pwm_value PWM value (0-4294967295)
  * @param direction 0 = forward, 1 = backward
  */
void Motor_SetSpeed_Left(uint32_t pwm_value, uint8_t direction)
{
    // Set direction pins according to Keyestudio shield logic
    // Forward: D6 LOW, D7 HIGH
    // Backward: D6 HIGH, D7 LOW
    if (direction == 0)  // Forward
    {
        HAL_GPIO_WritePin(MOTOR_LEFT_DIR1_PORT, MOTOR_LEFT_DIR1_PIN, GPIO_PIN_SET);   // D7 HIGH
        HAL_GPIO_WritePin(MOTOR_LEFT_DIR2_PORT, MOTOR_LEFT_DIR2_PIN, GPIO_PIN_RESET); // D6 LOW
    }
    else  // Backward
    {
        HAL_GPIO_WritePin(MOTOR_LEFT_DIR1_PORT, MOTOR_LEFT_DIR1_PIN, GPIO_PIN_RESET); // D7 LOW
        HAL_GPIO_WritePin(MOTOR_LEFT_DIR2_PORT, MOTOR_LEFT_DIR2_PIN, GPIO_PIN_SET);   // D6 HIGH
    }
    
    // Set PWM duty cycle
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value);
}

/**
  * @brief Stop motors
  */
void Motor_Stop(void)
{
    // Set PWM to 0
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    
    // Reset direction pins
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR1_PORT, MOTOR_RIGHT_DIR1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR2_PORT, MOTOR_RIGHT_DIR2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR1_PORT, MOTOR_LEFT_DIR1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR2_PORT, MOTOR_LEFT_DIR2_PIN, GPIO_PIN_RESET);
}

// ===== ENCODER FUNCTIONS =====

/**
  * @brief Initialize encoder for input capture mode
  */
void Encoder_Init_InputCapture(void)
{
    // Reset encoder data
    memset(&encoder_right, 0, sizeof(Encoder_Data_t));
    
    // Reset input capture variables
    ic_val1 = 0;
    ic_val2 = 0;
    ic_edge_count = 0;
    ic_ready = 0;
    
    UART_Printf("Encoder initialized for Input Capture mode on PA0 (TIM3_CH1)\r\n");
}

/**
  * @brief Calculate frequency from period
  * @param period_us Period in microseconds
  * @return Frequency in Hz
  */
float Encoder_Calculate_Frequency(uint32_t period_us)
{
    if (period_us == 0)
        return 0.0f;
    
    // Frequency = 1 / Period (in seconds)
    // Period in seconds = period_us / 1,000,000
    return 1000000.0f / (float)period_us;
}

/**
  * @brief Calculate RPM from frequency
  * @param frequency_hz Frequency in Hz
  * @return RPM
  */
float Encoder_Calculate_RPM(float frequency_hz)
{
    // RPM = (Pulses per second) * 60 / (Pulses per revolution)
    return (frequency_hz * 60.0f) / (float)PULSES_PER_REVOLUTION;
}

// ===== TASK 4: INPUT CAPTURE FUNCTIONS =====

/**
  * @brief Input Capture interrupt callback
  * This function is called automatically by HAL every time TIM3 CH1 captures a rising edge
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (ic_edge_count == 0)
        {
            // First rising edge captured
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            ic_edge_count = 1;
        }
        else if (ic_edge_count == 1)
        {
            // Second rising edge captured
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            ic_edge_count = 0;
            ic_ready = 1;  // Signal main loop that RPM is ready to calculate
            
            // Increment pulse count
            encoder_right.pulse_count++;
        }
    }
}

/**
  * @brief Calculate and print RPM from captured values
  */
void calculate_and_print_RPM(void)
{
    uint32_t ticks;
    float frequency;
    float rpm;
    
    // Calculate period in ticks (handling timer overflow)
    if (ic_val2 > ic_val1)
    {
        ticks = ic_val2 - ic_val1;
    }
    else
    {
        // Timer overflow occurred
        ticks = (65535 - ic_val1) + ic_val2 + 1;
    }
    
    // Store period
    encoder_right.pulse_period = ticks;
    
    // Check if period is valid
    if (ticks > MIN_VALID_PERIOD && ticks < MAX_VALID_PERIOD)
    {
        encoder_right.valid = 1;
        
        // Calculate frequency and RPM
        frequency = Encoder_Calculate_Frequency(ticks);
        rpm = Encoder_Calculate_RPM(frequency);
        
        // Print result
        UART_Printf("Period: %lu µs, Frequency: %.2f Hz, RPM: %.2f\r\n", 
                   ticks, frequency, rpm);
    }
    else
    {
        UART_Printf("Invalid period: %lu µs (out of range)\r\n", ticks);
    }
}

/**
  * @brief Print motor speed information for input capture mode
  */
void Print_Motor_Speed_InputCapture(void)
{
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Print every 500ms
    if (current_time - last_print_time >= 500)
    {
        last_print_time = current_time;
        
        // Clear screen
        UART_Printf("\033[2J\033[H");
        UART_Printf("========================================\r\n");
        UART_Printf("   LAB 6 TASK 4: INPUT CAPTURE MODE\r\n");
        UART_Printf("========================================\r\n\n");
        
        UART_Printf("Motor PWM: 50%% duty cycle\r\n");
        UART_Printf("Encoder PPR: %d\r\n", PULSES_PER_REVOLUTION);
        UART_Printf("Input Capture Pin: PA0 (TIM3_CH1)\r\n\n");
        
        if (encoder_right.valid && encoder_right.pulse_count > 0)
        {
            UART_Printf("--- MEASUREMENTS ---\r\n");
            UART_Printf("Pulse Count: %lu\r\n", encoder_right.pulse_count);
            UART_Printf("Last Period: %lu µs\r\n", encoder_right.pulse_period);
            
            if (encoder_right.pulse_period > MIN_VALID_PERIOD)
            {
                float freq = Encoder_Calculate_Frequency(encoder_right.pulse_period);
                float rpm = Encoder_Calculate_RPM(freq);
                UART_Printf("Frequency: %.2f Hz\r\n", freq);
                UART_Printf("Motor Speed: %.2f RPM\r\n", rpm);
                
                // Visual speed indicator
                UART_Printf("\r\nSpeed: [");
                int bars = (int)(rpm / 100.0f);
                if (bars > 50) bars = 50;
                for (int i = 0; i < bars; i++) UART_SendString("#");
                for (int i = bars; i < 50; i++) UART_SendString(".");
                UART_Printf("]\r\n");
            }
        }
        else
        {
            UART_Printf("⏳ Waiting for encoder pulses...\r\n");
            UART_Printf("Edge count: %d\r\n", ic_edge_count);
        }
        
        UART_Printf("\r\nTIM3 Counter: %lu\r\n", __HAL_TIM_GET_COUNTER(&htim3));
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
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_USB_PCD_Init();
    
    /* USER CODE BEGIN 2 */

    /* --- Motor Direction Setup --- */
    // Set motors to forward direction
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR1_PORT, MOTOR_RIGHT_DIR1_PIN, GPIO_PIN_SET);   // D12 HIGH
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR2_PORT, MOTOR_RIGHT_DIR2_PIN, GPIO_PIN_RESET); // D8 LOW
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR1_PORT, MOTOR_LEFT_DIR1_PIN, GPIO_PIN_SET);     // D7 HIGH
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR2_PORT, MOTOR_LEFT_DIR2_PIN, GPIO_PIN_RESET);   // D6 LOW

    /* --- Initialize Motor PWM --- */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Right motor PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Left motor PWM

    /* --- Set PWM Duty Cycle (50%) --- */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_TEST_SPEED);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_TEST_SPEED);

    /* --- Initialize Encoder for Input Capture --- */
    Encoder_Init_InputCapture();

    /* --- Start TIM3 Channel 1 Input Capture with Interrupt --- */
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

    /* --- UART Test Message --- */
    UART_Printf("\r\n\n========================================\r\n");
    UART_Printf("LAB 6 TASK 4: INPUT CAPTURE MODE\r\n");
    UART_Printf("Encoder connected to PA0 (TIM3_CH1)\r\n");
    UART_Printf("Motors running at 50%% duty cycle\r\n");
    UART_Printf("========================================\r\n\n");

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        
        // Check if RPM is ready to calculate
        if (ic_ready)
        {
            calculate_and_print_RPM();
            ic_ready = 0;
        }
        
        // Print speed information periodically
        Print_Motor_Speed_InputCapture();
        
        HAL_Delay(10);
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

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief TIM2 Initialization Function (for motor PWM)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;  // 32-bit max value
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    
    HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function (for Input Capture)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim3.Instance = TIM3;
    
    // Prescaler = 47 gives 1µs per tick with 48MHz timer clock
    // 48MHz / 48 = 1MHz = 1µs per tick
    htim3.Init.Prescaler = 47;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;  // 16-bit timer max
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Configure Channel 1 for Input Capture on rising edge
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    
    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Enable TIM3 interrupt in NVIC
    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{
    hpcd_USB_FS.Instance = USB;
    hpcd_USB_FS.Init.dev_endpoints = 8;
    hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
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

    /* Configure GPIO pin Output Level for onboard LEDs */
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

    /* Configure GPIO pin Output Level for motor direction pins */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

    /* Configure GPIO pins for onboard peripherals */
    GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Configure GPIO pins for onboard LEDs */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Configure USER button */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /* Configure PA0 as Input for TIM3_CH1 (Input Capture) */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function for TIM3
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;  // TIM3 alternate function
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure motor direction pins */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure left motor direction pins */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/**
  * @brief TIM3 MSP Post Initialization
  * @param htim TIM handle
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM3)
    {
        /* Enable TIM3 clock */
        __HAL_RCC_TIM3_CLK_ENABLE();
        
        /* Configure PA0 for TIM3_CH1 */
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        /* Enable TIM3 interrupt */
        HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
}

/**
  * @brief TIM2 MSP Post Initialization (for PWM pins)
  * @param htim TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM2)
    {
        /* Enable TIM2 clock */
        __HAL_RCC_TIM2_CLK_ENABLE();
        
        /* Configure PWM pins (PA1, PA2 for TIM2) */
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        
        // PA1 for TIM2_CH2 (Right motor PWM)
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        // PA2 for TIM2_CH3 (Left motor PWM)
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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