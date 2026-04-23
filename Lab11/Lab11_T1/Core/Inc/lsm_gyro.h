#ifndef LSM_GYRO_H
#define LSM_GYRO_H

#include "stm32f3xx_hal.h"
#include <math.h>

// ── LSM303AGR (I2C) ──────────────────────────────────────────────────────────
#define LSM_WRITE_ADDR      0x32
#define LSM_READ_ADDR       0x33
#define LSM_CTRL_REG1_A     0x20
#define LSM_CTRL_REG4_A     0x23
#define LSM_OUT_X_L_A       0x28
#define LSM_ACCEL_SCALE     3.9f        // mg/LSB in normal mode
#define LSM_MG_TO_G         0.001f      // mg → g

// ── I3G4250D / L3GD20 (SPI) ─────────────────────────────────────────────────
#define GYRO_CTRL_REG1      0x20
#define GYRO_CTRL_REG1_VAL  0x8F        // Power on, X/Y/Z enabled
#define GYRO_OUT_X_L        0x28
#define GYRO_SENSITIVITY    0.00875f    // dps/LSB for ±245 dps
#define GYRO_READ_FLAG      0x80        // MSB=1 for read
#define GYRO_AUTO_INC       0x40        // bit6=1 for auto-increment

// ── Complementary filter ─────────────────────────────────────────────────────
#define DT                  0.01f       // 100 Hz → 10ms per tick
#define RAD_TO_DEG          57.2958f

// ── Calibration duration ─────────────────────────────────────────────────────
// Board must be held still in its default resting position during this window.
// All angles after calibration will be reported relative to that position (0,0,0).
// Change this value to 10, 12, 15, or 20 seconds as needed.
#define CALIB_SECONDS       15

// ── Data struct ──────────────────────────────────────────────────────────────
typedef struct {
    int16_t raw_ax, raw_ay, raw_az;
    float   ax, ay, az;
    float   acc_angle;
    float   acc_offset;

    int16_t raw_gx, raw_gy, raw_gz;
    float   gx, gy, gz;
    float   gyro_offset;

    float   angle;
} IMU_t;

// Function prototypes
void IMU_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, IMU_t *imu);
void IMU_Offset(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, IMU_t *imu);
void IMU_Read(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, IMU_t *imu);
void IMU_Print(UART_HandleTypeDef *huart, IMU_t *imu);

#endif