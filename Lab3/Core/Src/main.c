/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * @file           : main.c

  * @brief          : Main program body - 7-Segment Display with All Tasks

  ******************************************************************************

  * @attention

  * 7-Segment Display: COMMON ANODE

  * Segment ON  = GPIO LOW  (0)

  * Segment OFF = GPIO HIGH (1)

  * USER Button (B1): Connected to PA0

  * External Button: Connected to PB5 (for Task 3)

  ******************************************************************************

  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "main.h"

#include <stdbool.h>

#include <stdlib.h>

#include <time.h>

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */

/* ===== 7-Segment Display Pin Mapping ===== */

#define SEG_A_PORT GPIOA

#define SEG_A_PIN  GPIO_PIN_1

#define SEG_B_PORT GPIOA

#define SEG_B_PIN  GPIO_PIN_4

#define SEG_C_PORT GPIOA

#define SEG_C_PIN  GPIO_PIN_5

#define SEG_D_PORT GPIOA

#define SEG_D_PIN  GPIO_PIN_6

#define SEG_E_PORT GPIOA

#define SEG_E_PIN  GPIO_PIN_7

#define SEG_F_PORT GPIOB

#define SEG_F_PIN  GPIO_PIN_0

#define SEG_G_PORT GPIOB

#define SEG_G_PIN  GPIO_PIN_1

/* Button Definitions */

#define USER_BUTTON_PORT GPIOA

#define USER_BUTTON_PIN  GPIO_PIN_0

#define EXT_BUTTON_PORT GPIOB

#define EXT_BUTTON_PIN  GPIO_PIN_5

/* Debug LED (onboard LED on STM32F3 Discovery) */

#define DEBUG_LED_PORT GPIOC

#define DEBUG_LED_PIN  GPIO_PIN_13

/* ===== SHARED VARIABLES ===== */

/* Segment patterns for Common Anode display (0=ON, 1=OFF) */

/* Bit order: .GFEDCBA (bit 0 = A, bit 6 = G) */

const uint8_t HEX_PATTERNS[16] = {

  0b00111111,  // 0: A,B,C,D,E,F ON

  0b00000110,  // 1: B,C ON

  0b01011011,  // 2: A,B,D,E,G ON

  0b01001111,  // 3: A,B,C,D,G ON

  0b01100110,  // 4: B,C,F,G ON

  0b01101101,  // 5: A,C,D,F,G ON

  0b01111101,  // 6: A,C,D,E,F,G ON

  0b00000111,  // 7: A,B,C ON

  0b01111111,  // 8: ALL ON

  0b01101111,  // 9: A,B,C,D,F,G ON

  0b01110111,  // A: A,B,C,E,F,G ON

  0b01111100,  // b: C,D,E,F,G ON

  0b00111001,  // C: A,D,E,F ON

  0b01011110,  // d: B,C,D,E,G ON

  0b01111001,  // E: A,D,E,F,G ON

  0b01110001   // F: A,E,F,G ON

};

/* ===== TASK 1: Hexadecimal Counter Variables ===== */

uint8_t hex_counter = 0;

/* ===== TASK 2: Student ID Display Variables ===== */

const uint8_t STUDENT_ID[] = {1, 0, 2, 7, 0};

const uint8_t ID_LENGTH = 5;

uint8_t current_digit_index = 0;

/* ===== TASK 3: Dual Button Counter Variables ===== */

int8_t dual_counter = 0;

const int8_t COUNTER_MIN = 0;

const int8_t COUNTER_MAX = 15;

/* ===== TASK 4: Random Digit Display Variables ===== */

uint8_t random_digit = 1;  // Initial random digit (1-6)

/* Button state variables (shared for Tasks 2, 3, 4) */

static bool user_button_was_pressed = false;

static bool ext_button_was_pressed = false;

static uint32_t user_button_press_time = 0;

static uint32_t ext_button_press_time = 0;

const uint32_t DEBOUNCE_DELAY = 200; // ms

/* Random seed initialization flag */

static bool random_initialized = false;

/* ===== DISPLAY FUNCTIONS ===== */

/* Display hex digit 0-F (Common Anode) */

void Display_Hex(uint8_t digit)

{

  if(digit > 0xF) return;

  

  uint8_t pattern = HEX_PATTERNS[digit];

  

  HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, (pattern & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, (pattern & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, (pattern & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, (pattern & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, (pattern & 0x10) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, (pattern & 0x20) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, (pattern & 0x40) ? GPIO_PIN_RESET : GPIO_PIN_SET);

}

/* Display digit 1-6 (for Task 4) */

void Display_Digit_1to6(uint8_t digit)

{

  if(digit < 1 || digit > 6) return;

  

  // Use the same patterns as hex display for 1-6

  Display_Hex(digit);  // Works because digits 1-6 are the same in hex patterns

}

/* Turn off all segments */

void Segments_AllOff(void)

{

  HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_SET);

  HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_SET);

}

/* ===== BUTTON FUNCTIONS ===== */

/* Check if button is pressed (with debouncing) - for Tasks 2, 3, 4 */

bool Is_Button_Pressed(GPIO_TypeDef* port, uint16_t pin, bool* was_pressed, uint32_t* press_time)

{

  uint8_t button_state = HAL_GPIO_ReadPin(port, pin);

  uint32_t current_time = HAL_GetTick();

  

  if (button_state == GPIO_PIN_SET && !(*was_pressed)) {

    if ((current_time - *press_time) > DEBOUNCE_DELAY) {

      *was_pressed = true;

      *press_time = current_time;

      return true;

    }

  }

  

  if (button_state == GPIO_PIN_RESET) {

    *was_pressed = false;

  }

  

  return false;

}

/* ===== TASK 4 SPECIFIC FUNCTIONS ===== */

/* Initialize random number generator using timer */

void Init_Random(void)

{

  if (!random_initialized) {

    // Use system timer tick as seed

    srand(HAL_GetTick());

    random_initialized = true;

  }

}

/* Generate random number between 1 and 6 */

uint8_t Generate_Random_1to6(void)

{

  // Generate random number between 1 and 6

  return (rand() % 6) + 1;  // rand() % 6 gives 0-5, +1 gives 1-6

}

/* Visual effect for dice roll */

void Dice_Roll_Effect(void)

{

  // Quick cycle through numbers 1-6

  for(int i = 0; i < 3; i++) {

    for(uint8_t num = 1; num <= 6; num++) {

      Display_Digit_1to6(num);

      HAL_Delay(30);

    }

  }

}

/* ===== DEBUG FUNCTIONS ===== */

/* Toggle debug LED */

void Toggle_Debug_LED(void)

{

  HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);

}

/* Blink LED */

void Blink_LED(uint32_t duration_ms)

{

  HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);

  HAL_Delay(duration_ms);

  HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);

}

/* Test function to identify each segment */

void Test_Segments(void)

{

  // Test each segment

  uint8_t test_patterns[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};

  

  for(int i = 0; i < 7; i++) {

    Segments_AllOff();

    HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, (test_patterns[i] & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, (test_patterns[i] & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, (test_patterns[i] & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, (test_patterns[i] & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, (test_patterns[i] & 0x10) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, (test_patterns[i] & 0x20) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, (test_patterns[i] & 0x40) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_Delay(500);

  }

  

  // Show all segments (number 8)

  Display_Hex(8);

  HAL_Delay(1000);

  

  Segments_AllOff();

}

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

/**

  * @brief  The application entry point.

  * @retval int

  */

int main(void)

{

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  /* Configure the system clock */

  SystemClock_Config();

  /* Initialize all configured peripherals */

  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  

  // Initialize debug LED

  HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);

  

  // Initialize random number generator for Task 4

  Init_Random();

  

  // Initialize with first random digit for Task 4

  random_digit = Generate_Random_1to6();

  Display_Digit_1to6(random_digit);

  

  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)

  {

    /* ===== UNCOMMENT THE TASK YOU WANT TO RUN ===== */

    

    /***** TASK 1: Hexadecimal Counter (0-F) AUTO-CYCLE *****/

    /*

    Display_Hex(hex_counter);

    HAL_Delay(2000);

    hex_counter = (hex_counter + 1) & 0x0F;  // Wrap around after F

    */

    

    /***** TASK 2: Student ID Display on Button Press *****/

    /*

    if (Is_Button_Pressed(USER_BUTTON_PORT, USER_BUTTON_PIN, 

                          &user_button_was_pressed, &user_button_press_time)) {

      current_digit_index = (current_digit_index + 1) % ID_LENGTH;

      Display_Hex(STUDENT_ID[current_digit_index]);

      Blink_LED(100);

    }

    HAL_Delay(10);

    */

    

    /***** TASK 3: Dual Button Counter (Increment/Decrement) - NO WRAPAROUND *****/

    /*

    // USER Button (PA0): Increment (+1)

    if (Is_Button_Pressed(USER_BUTTON_PORT, USER_BUTTON_PIN, 

                          &user_button_was_pressed, &user_button_press_time)) {

      if (dual_counter < COUNTER_MAX) {

        dual_counter++;

        Display_Hex(dual_counter);

        Blink_LED(50);

      }

    }

    

    // EXTERNAL Button (PB5): Decrement (-1)

    if (Is_Button_Pressed(EXT_BUTTON_PORT, EXT_BUTTON_PIN, 

                          &ext_button_was_pressed, &ext_button_press_time)) {

      if (dual_counter > COUNTER_MIN) {

        dual_counter--;

        Display_Hex(dual_counter);

        Blink_LED(50);

      }

    }

    

    HAL_Delay(10);

    */

    

    /***** TASK 4: Random Digit Display (1-6) on Button Press *****/

    // USER Button generates new random number 1-6

    if (Is_Button_Pressed(USER_BUTTON_PORT, USER_BUTTON_PIN, 

                          &user_button_was_pressed, &user_button_press_time)) {

      // Optional: Show dice rolling effect

      // Dice_Roll_Effect();

      

      // Generate new random number

      random_digit = Generate_Random_1to6();

      

      // Display the new random digit

      Display_Digit_1to6(random_digit);

      

      // Visual feedback

      Blink_LED(100);

      

      // Brief pause to prevent multiple rapid presses

      HAL_Delay(200);

    }

    

    HAL_Delay(10);

    

    /* USER CODE END WHILE */

  }

}

/**

  * @brief System Clock Configuration

  * @retval None

  */

void SystemClock_Config(void)

{

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;

  RCC_OscInitStruct.HSIState = RCC_HSI_ON;

  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)

  {

    Error_Handler();

  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK

                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)

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

  __HAL_RCC_GPIOA_CLK_ENABLE();

  __HAL_RCC_GPIOB_CLK_ENABLE();

  __HAL_RCC_GPIOC_CLK_ENABLE();  // For debug LED

  /* Configure GPIO pin Output Level - All segments OFF initially (HIGH for common anode) */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  

  /* Configure Debug LED (PC13) as output */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Start with LED OFF

  /* Configure GPIO pins : PA1 PA4 PA5 PA6 PA7 (7-Segment segments) */

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pins : PB0 PB1 (7-Segment segments) */

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  

  /* Configure Debug LED (PC13) */

  GPIO_InitStruct.Pin = GPIO_PIN_13;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  

  /* Configure USER Button (PA0) as input with INTERNAL PULL-DOWN */

  GPIO_InitStruct.Pin = GPIO_PIN_0;

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // Button connects to 3.3V

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  

  /* Configure EXTERNAL Button (PB5) as input with INTERNAL PULL-DOWN */

  GPIO_InitStruct.Pin = GPIO_PIN_5;

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // Button connects to 3.3V

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)

{

}

#endif /* USE_FULL_ASSERT */