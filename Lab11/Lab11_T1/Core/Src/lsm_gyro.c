#include "lsm_gyro.h"
#include <stdio.h>
#include <string.h>

// ── SPI helper: read N bytes from gyro starting at reg ───────────────────────
static void gyro_read_regs(SPI_HandleTypeDef *hspi, uint8_t reg,
                            uint8_t *buf, uint8_t len) {
    uint8_t cmd = reg | GYRO_READ_FLAG | GYRO_AUTO_INC;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(hspi, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, buf, len, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);   // CS HIGH
}

// ── Init ─────────────────────────────────────────────────────────────────────
void IMU_Init(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, IMU_t *imu) {
    uint8_t data;

    // --- LSM303AGR accelerometer (I2C) ---
    data = 0x67;  // ODR=100Hz, normal mode, XYZ enabled
    HAL_I2C_Mem_Write(hi2c, LSM_WRITE_ADDR, LSM_CTRL_REG1_A, 1,
                      &data, 1, HAL_MAX_DELAY);

    data = 0x00;  // Normal power mode
    HAL_I2C_Mem_Write(hi2c, LSM_WRITE_ADDR, LSM_CTRL_REG4_A, 1,
                      &data, 1, HAL_MAX_DELAY);

    // --- On-board gyroscope (SPI) ---
    uint8_t tx[2] = { GYRO_CTRL_REG1, GYRO_CTRL_REG1_VAL };
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    imu->angle = 0.0f;

    HAL_Delay(100);
}

// ── Read raw accelerometer from LSM303AGR ────────────────────────────────────
static void read_accel(I2C_HandleTypeDef *hi2c, IMU_t *imu) {
    uint8_t buf[6];

    HAL_I2C_Mem_Read(hi2c, LSM_READ_ADDR, LSM_OUT_X_L_A | 0x80, 1,
                     buf, 6, HAL_MAX_DELAY);

    imu->raw_ax = (int16_t)(buf[1] << 8 | buf[0]);
    imu->raw_ay = (int16_t)(buf[3] << 8 | buf[2]);
    imu->raw_az = (int16_t)(buf[5] << 8 | buf[4]);

    imu->ax = (imu->raw_ax * LSM_ACCEL_SCALE) * LSM_MG_TO_G;
    imu->ay = (imu->raw_ay * LSM_ACCEL_SCALE) * LSM_MG_TO_G;
    imu->az = (imu->raw_az * LSM_ACCEL_SCALE) * LSM_MG_TO_G;

    imu->acc_angle = atan2f(imu->ay, imu->az) * RAD_TO_DEG;
    imu->acc_angle -= imu->acc_offset;
}

// ── Read raw gyroscope (on-board SPI) ────────────────────────────────────────
static void read_gyro(SPI_HandleTypeDef *hspi, IMU_t *imu) {
    uint8_t buf[6];

    gyro_read_regs(hspi, GYRO_OUT_X_L, buf, 6);

    imu->raw_gx = (int16_t)(buf[1] << 8 | buf[0]);
    imu->raw_gy = (int16_t)(buf[3] << 8 | buf[2]);
    imu->raw_gz = (int16_t)(buf[5] << 8 | buf[4]);

    imu->gx = imu->raw_gx * GYRO_SENSITIVITY;
    imu->gy = imu->raw_gy * GYRO_SENSITIVITY;
    imu->gz = imu->raw_gz * GYRO_SENSITIVITY;

    imu->gx -= imu->gyro_offset;
}

// ── Offset calibration (timed window) ────────────────────────────────────────
// Place the board in its default/resting position.
// This function samples for CALIB_SECONDS seconds, averages
// all readings, and sets that as the zero reference.
// After this, all angles are reported relative to that position.
void IMU_Offset(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, IMU_t *imu) {
    float acc_sum  = 0.0f;
    float gyro_sum = 0.0f;
    int   samples  = 0;

    uint32_t start = HAL_GetTick();                        // current time in ms
    uint32_t duration = CALIB_SECONDS * 1000;              // convert to ms

    while ((HAL_GetTick() - start) < duration) {
        uint8_t buf[6];

        // Read accelerometer
        HAL_I2C_Mem_Read(hi2c, LSM_READ_ADDR, LSM_OUT_X_L_A | 0x80, 1,
                         buf, 6, HAL_MAX_DELAY);
        int16_t raw_ay = (int16_t)(buf[3] << 8 | buf[2]);
        int16_t raw_az = (int16_t)(buf[5] << 8 | buf[4]);
        float ay = (raw_ay * LSM_ACCEL_SCALE) * LSM_MG_TO_G;
        float az = (raw_az * LSM_ACCEL_SCALE) * LSM_MG_TO_G;
        acc_sum += atan2f(ay, az) * RAD_TO_DEG;

        // Read gyroscope
        gyro_read_regs(hspi, GYRO_OUT_X_L, buf, 6);
        int16_t raw_gx = (int16_t)(buf[1] << 8 | buf[0]);
        gyro_sum += raw_gx * GYRO_SENSITIVITY;

        samples++;
        HAL_Delay(10);  // sample every 10ms during calibration
    }

    // Store averages as the zero reference
    imu->acc_offset  = acc_sum  / samples;
    imu->gyro_offset = gyro_sum / samples;

    // Reset angle to 0 — all future readings are relative to this position
    imu->angle = 0.0f;
}

// ── Read + complementary filter ──────────────────────────────────────────────
void IMU_Read(I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, IMU_t *imu) {
    read_accel(hi2c, imu);
    read_gyro(hspi, imu);

    // angle = 0.98 * (angle + gyro * dt) + 0.02 * acc_angle
    imu->angle = 0.98f * (imu->angle + imu->gx * DT)
               + 0.02f * imu->acc_angle;
}

// ── Print (comma-separated for sensorplot.py) ────────────────────────────────
void IMU_Print(UART_HandleTypeDef *huart, IMU_t *imu) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\r\n",
                       imu->acc_angle,
                       imu->gx,
                       imu->angle);
    HAL_UART_Transmit(huart, (uint8_t *)buf, len, HAL_MAX_DELAY);
}