#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "lsm6dsr_reg.h"
#include "lsm6dsr.h"
#include "lps22hh.h"
#include <stdint.h>

#define IMU_SAMPLING_FREQ (417)
#define IMU_SAMPLING_PERIOD_MS (1000.0 / IMU_SAMPLING_FREQ)

#define IMU_CS_PORT  GPIOA
#define IMU_CS_PIN   GPIO_PIN_8
#define BARO_CS_PORT GPIOC
#define BARO_CS_PIN  GPIO_PIN_13

#define VL53L0X_I2C_ADDR_7BIT   (0x29U)
#define VL53L0X_I2C_ADDR_8BIT   (VL53L0X_I2C_ADDR_7BIT << 1)

#define IMU_TASK_PERIOD_S  (10.0f / 1000.0f)

#define GRAVITY_MPS2  (9.80665f)
#define DEG_TO_RAD    (0.01745329252f)
#define SEA_LEVEL_HPA (1013.25f)

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;
typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} spi_device_t;

typedef struct
{
    uint32_t timestamp_us;

    int32_t accel_x_mg;
    int32_t accel_y_mg;
    int32_t accel_z_mg;

    int32_t gyro_x_mdps;
    int32_t gyro_y_mdps;
    int32_t gyro_z_mdps;

    uint8_t accel_ready;
    uint8_t gyro_ready;
} raw_imu_sample_t;

typedef struct
{
    uint32_t timestamp_us;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    /* Acceleromter-driven measurements of RP*/
    float accel_roll;
    float accel_pitch;

    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;

    float vel_x_mps;
    float vel_y_mps;
    float vel_z_mps;

    float pos_x_m;
    float pos_y_m;
    float pos_z_m;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float gyro_x_rps;
    float gyro_y_rps;
    float gyro_z_rps;

    float gyro_x_rad_abs;
    float gyro_y_rad_abs;
    float gyro_z_rad_abs;

    float combined_roll;
    float combined_pitch;
} processed_imu_sample_t;

typedef struct
{
    uint32_t timestamp_us;

    float pressure_hpa;
    float temperature_c;

    uint8_t pressure_ready;
    uint8_t temperature_ready;
} raw_baro_sample_t;

typedef struct
{
    uint32_t timestamp_us;

    float pressure_hpa;
    float temperature_c;
    float altitude_m;
} processed_baro_sample_t;

typedef enum
{
    DEBUG_OUT_RAW_IMU = 0,
    DEBUG_OUT_PROC_IMU,
    DEBUG_OUT_PROC_BARO,
    DEBUG_OUT_STAT
} debug_output_mode_t;

typedef struct
{
    uint32_t timestamp_us;
    uint16_t range_mm;
    uint8_t range_status;
    uint8_t valid;
} raw_tof_sample_t;

typedef struct
{
    uint32_t timestamp_us;
    float range_m;
    uint8_t range_status;
    uint8_t valid;
} processed_tof_sample_t;

extern processed_tof_sample_t g_processed_tof;

int32_t SensorManager_Init(void);
void SensorManager_RunOnce(void);
int32_t MX_LSM6DSR_Init(void);
int32_t MX_LPS22HH_Init(void);
//void vl53l0x_acquire_one_sample(void);
int32_t vl53l0x_api_init_device(void);
void vl53l0x_cfg_INT(void);
void vl53l0x_start_next_measurement(void);


#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
