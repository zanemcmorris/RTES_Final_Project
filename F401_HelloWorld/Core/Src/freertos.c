/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include <stdbool.h>
#include "lsm6dsr.h"
#include "lps22hh.h"
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include "vl53l0x_platform.h"

#define USER_LED_1_PORT (GPIOB)
#define USER_LED_2_PORT (GPIOB)
#define USER_LED_1_PIN  (GPIO_PIN_4)
#define USER_LED_2_PIN  (GPIO_PIN_5)

#define IMU_CS_PORT  GPIOA
#define IMU_CS_PIN   GPIO_PIN_8
#define BARO_CS_PORT GPIOC
#define BARO_CS_PIN  GPIO_PIN_13

#define VL53L0X_I2C_ADDR_7BIT   (0x29U)
#define VL53L0X_I2C_ADDR_8BIT   (VL53L0X_I2C_ADDR_7BIT << 1)

#define IMU_TASK_PERIOD_MS (10)
#define IMU_TASK_PERIOD_S  (10.0f / 1000.0f)

#define GRAVITY_MPS2  (9.80665f)
#define DEG_TO_RAD    (0.01745329252f)
#define SEA_LEVEL_HPA (1013.25f)

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c2;  // for lidar
extern TIM_HandleTypeDef htim4;
static void imu_stats_send_line(void);
static void baro_overrun_send_line(void);



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

	float accel_x_mps2;
	float accel_y_mps2;
	float accel_z_mps2;

	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;

	float gyro_x_rps;
	float gyro_y_rps;
	float gyro_z_rps;
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


/*typedef enum
{
    DEBUG_OUT_RAW_IMU = 0,
    DEBUG_OUT_PROC_IMU,
    DEBUG_OUT_PROC_BARO
} debug_output_mode_t; */

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

static volatile debug_output_mode_t g_debug_output_mode = DEBUG_OUT_STAT;  // chnage this to whicheverr macro absed on what u wanna see

static LSM6DSR_Object_t MotionSensor;
static LPS22HH_Object_t BaroSensor;
static VL53L0X_Dev_t g_vl53l0x_dev;

static spi_device_t imu_dev  = { .hspi = &hspi2, .cs_port = IMU_CS_PORT,  .cs_pin = IMU_CS_PIN };
static spi_device_t baro_dev = { .hspi = &hspi2, .cs_port = BARO_CS_PORT, .cs_pin = BARO_CS_PIN };

static raw_imu_sample_t       g_raw_imu       = {0};
static processed_imu_sample_t g_processed_imu = {0};
static raw_baro_sample_t      g_raw_baro      = {0};
static processed_baro_sample_t g_processed_baro = {0};

static float g_gyro_bias_x_mdps = 0.0f;
static float g_gyro_bias_y_mdps = 0.0f;
static float g_gyro_bias_z_mdps = 0.0f;

static uint32_t g_imu_task_loops  = 0;
static uint32_t g_acc_ready_count = 0;
static uint32_t g_gyro_ready_count = 0;
static uint32_t g_acc_read_count  = 0;
static uint32_t g_gyro_read_count = 0;


static uint32_t g_baro_status_checks = 0;
static uint32_t g_baro_press_overrun_count = 0;
static uint32_t g_baro_temp_overrun_count = 0;


static uint32_t g_imu_fifo_status_checks = 0;
static uint32_t g_imu_fifo_overrun_count = 0;
static uint32_t g_imu_fifo_max_samples = 0;
static uint16_t g_imu_fifo_last_samples = 0;

static uint32_t g_imu_fifo_drained_samples = 0;

static raw_tof_sample_t g_raw_tof = {0};
static processed_tof_sample_t g_processed_tof = {0};

//static float g_latest_valid_tof_m = 0.0f;
//static uint8_t g_latest_valid_tof_available = 0U;

//void ToFAcquisitionTask(void *argument);
static inline void setUserLEDOne(uint8_t state)
{
	HAL_GPIO_WritePin(USER_LED_1_PORT, USER_LED_1_PIN, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline void setUserLEDTwo(uint8_t state)
{
	HAL_GPIO_WritePin(USER_LED_2_PORT, USER_LED_2_PIN, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline uint32_t micros(void)
{
	return (uint32_t)((HAL_GetTick() * 1000U) +
			((SysTick->LOAD - SysTick->VAL) * 1000U) / (SysTick->LOAD + 1U));
}

static void spi2_deselect_all(void)
{
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);
}

static void spi_dev_select(spi_device_t *dev)
{
	spi2_deselect_all();
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void spi_dev_deselect(spi_device_t *dev)
{
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static int32_t BSP_SPI2_Init(void)
{
	spi2_deselect_all();
	return 0;
}

static int32_t BSP_SPI2_DeInit(void)
{
	spi2_deselect_all();
	return 0;
}

static int32_t spi_bus_read_reg(spi_device_t *dev, uint8_t reg, uint8_t *pData, uint16_t Length)
{
	uint8_t txbuf[32];
	uint8_t rxbuf[32];

	if (Length == 0 || Length > (sizeof(txbuf) - 1))
		return LSM6DSR_ERROR;

	txbuf[0] = reg | 0x80;
	memset(&txbuf[1], 0x00, Length);
	memset(rxbuf, 0x00, Length + 1);

	spi_dev_select(dev);

	if (HAL_SPI_TransmitReceive(dev->hspi, txbuf, rxbuf, Length + 1, 100) != HAL_OK)
	{
		spi_dev_deselect(dev);
		return LSM6DSR_ERROR;
	}

	spi_dev_deselect(dev);
	memcpy(pData, &rxbuf[1], Length);
	return LSM6DSR_OK;
}

static int32_t spi_bus_write_reg(spi_device_t *dev, uint8_t reg, uint8_t *pData, uint16_t Length)
{
	uint8_t txbuf[32];

	if (Length == 0 || Length > (sizeof(txbuf) - 1))
		return LSM6DSR_ERROR;

	txbuf[0] = reg;
	memcpy(&txbuf[1], pData, Length);

	spi_dev_select(dev);

	if (HAL_SPI_Transmit(dev->hspi, txbuf, Length + 1, 100) != HAL_OK)
	{
		spi_dev_deselect(dev);
		return LSM6DSR_ERROR;
	}

	spi_dev_deselect(dev);
	return LSM6DSR_OK;
}

/* IMU-specific wrappers */
static int32_t IMU_SPI2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	(void)Addr;
	return spi_bus_read_reg(&imu_dev, (uint8_t)Reg, pData, Length);
}

static int32_t IMU_SPI2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	(void)Addr;
	return spi_bus_write_reg(&imu_dev, (uint8_t)Reg, pData, Length);
}

/* Barometer-specific wrappers */
static int32_t BARO_SPI2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	(void)Addr;
	return spi_bus_read_reg(&baro_dev, (uint8_t)Reg, pData, Length);
}

static int32_t BARO_SPI2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	(void)Addr;
	return spi_bus_write_reg(&baro_dev, (uint8_t)Reg, pData, Length);
}

static HAL_StatusTypeDef vl53l0x_read_u8(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            VL53L0X_I2C_ADDR_8BIT,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            100);
}

static void vl53l0x_setup_device_struct(void)
{
    memset(&g_vl53l0x_dev, 0, sizeof(g_vl53l0x_dev));
    g_vl53l0x_dev.I2cDevAddr = VL53L0X_I2C_ADDR_8BIT;
    g_vl53l0x_dev.comms_type = 1;
    g_vl53l0x_dev.comms_speed_khz = 400;
}



static int32_t MX_LSM6DSR_Init(void)
{
	LSM6DSR_IO_t io_ctx;
	uint8_t id = 0;
	int32_t fullScale = 2;

	io_ctx.Init     = BSP_SPI2_Init;
	io_ctx.DeInit   = BSP_SPI2_DeInit;
	io_ctx.BusType  = LSM6DSR_SPI_4WIRES_BUS;
	io_ctx.Address  = 0;
	io_ctx.WriteReg = IMU_SPI2_WriteReg;
	io_ctx.ReadReg  = IMU_SPI2_ReadReg;

	if (LSM6DSR_RegisterBusIO(&MotionSensor, &io_ctx) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_Init(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_ReadID(&MotionSensor, &id) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (id != 0x6B)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_GYRO_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_SetOutputDataRate(&MotionSensor, 104.0f) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_ACC_SetFullScale(&MotionSensor, fullScale) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_GYRO_SetOutputDataRate(&MotionSensor, 104.0f) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_GYRO_SetFullScale(&MotionSensor, LSM6DSR_250dps) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	return LSM6DSR_OK;
}

static int32_t MX_LSM6DSR_FIFO_Test_Init(void)
{
    if (lsm6dsr_fifo_mode_set(&(MotionSensor.Ctx), LSM6DSR_BYPASS_MODE) != LSM6DSR_OK)
        return LSM6DSR_ERROR;

    if (lsm6dsr_fifo_xl_batch_set(&(MotionSensor.Ctx), LSM6DSR_XL_BATCHED_AT_104Hz) != LSM6DSR_OK)
        return LSM6DSR_ERROR;

    if (lsm6dsr_fifo_gy_batch_set(&(MotionSensor.Ctx), LSM6DSR_GY_BATCHED_AT_104Hz) != LSM6DSR_OK)
        return LSM6DSR_ERROR;

    if (lsm6dsr_fifo_mode_set(&(MotionSensor.Ctx), LSM6DSR_STREAM_MODE) != LSM6DSR_OK)
        return LSM6DSR_ERROR;

    return LSM6DSR_OK;
}


static int32_t MX_LPS22HH_Init(void)
{
	LPS22HH_IO_t io_ctx;
	uint8_t id = 0;

	io_ctx.Init     = BSP_SPI2_Init;
	io_ctx.DeInit   = BSP_SPI2_DeInit;
	io_ctx.BusType  = LPS22HH_SPI_4WIRES_BUS;
	io_ctx.Address  = 0;
	io_ctx.WriteReg = BARO_SPI2_WriteReg;
	io_ctx.ReadReg  = BARO_SPI2_ReadReg;
	io_ctx.Delay    = HAL_Delay;

	if (LPS22HH_RegisterBusIO(&BaroSensor, &io_ctx) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_Init(&BaroSensor) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_ReadID(&BaroSensor, &id) != LPS22HH_OK)
		return LPS22HH_ERROR;

	if (id != 0xB3)
		return LPS22HH_ERROR;

	if (LPS22HH_PRESS_Enable(&BaroSensor) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_TEMP_Enable(&BaroSensor) != LPS22HH_OK)
		return LPS22HH_ERROR;

	if (lps22hh_lp_bandwidth_set(&(BaroSensor.Ctx), LPS22HH_LPF_ODR_DIV_9) != LPS22HH_OK)
		return LPS22HH_ERROR;

	if (LPS22HH_PRESS_SetOutputDataRate(&BaroSensor, 10.0f) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_TEMP_SetOutputDataRate(&BaroSensor, 10.0f) != LPS22HH_OK)
		return LPS22HH_ERROR;

	return LPS22HH_OK;
}

static VL53L0X_Error vl53l0x_api_init_device(void)
{
    VL53L0X_Error status;
    uint8_t vhv_settings = 0;
    uint8_t phase_cal = 0;
    uint32_t ref_spad_count = 0;
    uint8_t is_aperture_spads = 0;

    vl53l0x_setup_device_struct();

    status = VL53L0X_DataInit(&g_vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    status = VL53L0X_StaticInit(&g_vl53l0x_dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    status = VL53L0X_PerformRefSpadManagement(&g_vl53l0x_dev,
                                              &ref_spad_count,
                                              &is_aperture_spads);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    status = VL53L0X_PerformRefCalibration(&g_vl53l0x_dev,
                                           &vhv_settings,
                                           &phase_cal);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    status = VL53L0X_SetDeviceMode(&g_vl53l0x_dev,
                                   VL53L0X_DEVICEMODE_SINGLE_RANGING);
    return status;
}

static void preprocess_imu_sample(const raw_imu_sample_t *raw, processed_imu_sample_t *proc)
{
	proc->timestamp_us = raw->timestamp_us;

	proc->accel_x_g = raw->accel_x_mg / 1000.0f;
	proc->accel_y_g = raw->accel_y_mg / 1000.0f;
	proc->accel_z_g = raw->accel_z_mg / 1000.0f;

	proc->accel_x_mps2 = proc->accel_x_g * GRAVITY_MPS2;
	proc->accel_y_mps2 = proc->accel_y_g * GRAVITY_MPS2;
	proc->accel_z_mps2 = proc->accel_z_g * GRAVITY_MPS2;

	proc->gyro_x_dps = (raw->gyro_x_mdps - g_gyro_bias_x_mdps) / 1000.0f;
	proc->gyro_y_dps = (raw->gyro_y_mdps - g_gyro_bias_y_mdps) / 1000.0f;
	proc->gyro_z_dps = (raw->gyro_z_mdps - g_gyro_bias_z_mdps) / 1000.0f;

	proc->gyro_x_rps = proc->gyro_x_dps * DEG_TO_RAD;
	proc->gyro_y_rps = proc->gyro_y_dps * DEG_TO_RAD;
	proc->gyro_z_rps = proc->gyro_z_dps * DEG_TO_RAD;
}

static float pressure_to_altitude_m(float pressure_hpa)
{
	if (pressure_hpa <= 0.0f)
	{
		return 0.0f;
	}

	return 44330.0f * (1.0f - powf(pressure_hpa / SEA_LEVEL_HPA, 0.1903f));
}

static void preprocess_baro_sample(const raw_baro_sample_t *raw, processed_baro_sample_t *proc)
{
	proc->timestamp_us = raw->timestamp_us;
	proc->pressure_hpa = raw->pressure_hpa;
	proc->temperature_c = raw->temperature_c;
	proc->altitude_m = pressure_to_altitude_m(raw->pressure_hpa);
}

static void preprocess_tof_sample(const raw_tof_sample_t *raw, processed_tof_sample_t *proc)
{
    proc->timestamp_us = raw->timestamp_us;
    proc->range_status = raw->range_status;
    proc->valid = raw->valid;
    proc->range_m = raw->range_mm / 1000.0f;
}
static void imu_uart_send_raw_line(const LSM6DSR_Axes_t *accel_raw, const LSM6DSR_Axes_t *gyro_raw)
{
	char buf[128];

	int len = snprintf(buf, sizeof(buf),
			"IMU,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
			accel_raw->x / 1000.0f,
			accel_raw->y / 1000.0f,
			accel_raw->z / 1000.0f,
			gyro_raw->x / 1000.0f,
			gyro_raw->y / 1000.0f,
			gyro_raw->z / 1000.0f);

	if (len > 0)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
	}
}

static void imu_uart_send_processed_line(const processed_imu_sample_t *imu)
{
	char buf[160];

	int len = snprintf(buf, sizeof(buf),
			"PROC,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
			imu->accel_x_mps2,
			imu->accel_y_mps2,
			imu->accel_z_mps2,
			imu->gyro_x_rps,
			imu->gyro_y_rps,
			imu->gyro_z_rps);

	if (len > 0)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
	}
}

static void baro_uart_send_processed_line(const processed_baro_sample_t *baro)
{
	char buf[128];

	int len = snprintf(buf, sizeof(buf),
			"BAROP,%.2f,%.2f,%.2f\r\n",
			baro->pressure_hpa,
			baro->temperature_c,
			baro->altitude_m);

	if (len > 0)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
	}
}

static void debug_output_send_current_mode(const LSM6DSR_Axes_t *accel,
                                           const LSM6DSR_Axes_t *gyro,
                                           const processed_imu_sample_t *imu_proc,
                                           const processed_baro_sample_t *baro_proc)
{
    switch (g_debug_output_mode)
    {
        case DEBUG_OUT_RAW_IMU:
            imu_uart_send_raw_line(accel, gyro);
            break;

        case DEBUG_OUT_PROC_IMU:
            imu_uart_send_processed_line(imu_proc);
            break;

        case DEBUG_OUT_PROC_BARO:
            baro_uart_send_processed_line(baro_proc);
            break;

        case DEBUG_OUT_STAT:
            imu_stats_send_line();
            break;

        default:
            break;
    }
}

static void imu_stats_send_line(void)
{
    char buf[160];

    int len = snprintf(buf, sizeof(buf),
            "STAT,%lu,%lu,%lu,%lu,%lu\r\n",
            g_imu_task_loops,
            g_acc_ready_count,
            g_gyro_ready_count,
            g_acc_read_count,
            g_gyro_read_count);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}

static void baro_overrun_send_line(void)
{
    char buf[128];

    int len = snprintf(buf, sizeof(buf),
            "BOR,%lu,%lu,%lu\r\n",
            g_baro_status_checks,
            g_baro_press_overrun_count,
            g_baro_temp_overrun_count);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}

static void imu_fifo_send_line(void)
{
    char buf[128];

    int len = snprintf(buf, sizeof(buf),
            "IFIFO,%lu,%u,%lu,%lu\r\n",
            g_imu_fifo_status_checks,
            g_imu_fifo_last_samples,
            g_imu_fifo_max_samples,
            g_imu_fifo_overrun_count);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}


static void vl53l0x_uart_send_line(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

static void tof_uart_send_processed_line(const processed_tof_sample_t *tof)
{
    char buf[128];

    int len = snprintf(buf, sizeof(buf),
            "TOF,%lu,%.3f,%u,%u\r\n",
            (unsigned long)tof->timestamp_us,
            tof->range_m,
            (unsigned int)tof->range_status,
            (unsigned int)tof->valid);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
}

static void vl53l0x_basic_i2c_test(void)
{
    char msg[128];
    uint8_t value = 0;
    HAL_StatusTypeDef status;

    snprintf(msg, sizeof(msg), "\r\n[VL53L0X] basic I2C test start\r\n");
    vl53l0x_uart_send_line(msg);

    status = HAL_I2C_IsDeviceReady(&hi2c2, VL53L0X_I2C_ADDR_8BIT, 3, 100);
    if (status != HAL_OK)
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X] device not ready, status=%d\r\n",
                 (int)status);
        vl53l0x_uart_send_line(msg);
        return;
    }

    snprintf(msg, sizeof(msg),
             "[VL53L0X] device ACKed at 0x%02X\r\n",
             VL53L0X_I2C_ADDR_8BIT);
    vl53l0x_uart_send_line(msg);

    status = vl53l0x_read_u8(0xC0, &value);
    if (status == HAL_OK)
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X] reg 0xC0 = 0x%02X\r\n",
                 value);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X] read 0xC0 failed, status=%d\r\n",
                 (int)status);
    }
    vl53l0x_uart_send_line(msg);

    status = vl53l0x_read_u8(0xC1, &value);
    if (status == HAL_OK)
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X] reg 0xC1 = 0x%02X\r\n",
                 value);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X] read 0xC1 failed, status=%d\r\n",
                 (int)status);
    }
    vl53l0x_uart_send_line(msg);

    status = vl53l0x_read_u8(0xC2, &value);
    if (status == HAL_OK)
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X] reg 0xC2 = 0x%02X\r\n",
                 value);
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X] read 0xC2 failed, status=%d\r\n",
                 (int)status);
    }
    vl53l0x_uart_send_line(msg);
}

static void vl53l0x_api_init_test(void)
{
    char msg[128];
    VL53L0X_Error status;

    status = vl53l0x_api_init_device();

    if (status == VL53L0X_ERROR_NONE)
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X API] init OK\r\n");
    }
    else
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X API] init failed, status=%d\r\n",
                 (int)status);
    }

    vl53l0x_uart_send_line(msg);
}

static void vl53l0x_single_range_test(void)
{
    char msg[160];
    VL53L0X_Error status;
    VL53L0X_RangingMeasurementData_t measurement;

    status = vl53l0x_api_init_device();
    if (status != VL53L0X_ERROR_NONE)
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X RANGE] init failed, status=%d\r\n",
                 (int)status);
        vl53l0x_uart_send_line(msg);
        return;
    }

    status = VL53L0X_PerformSingleRangingMeasurement(&g_vl53l0x_dev, &measurement);
    if (status != VL53L0X_ERROR_NONE)
    {
        snprintf(msg, sizeof(msg),
                 "[VL53L0X RANGE] measure failed, status=%d\r\n",
                 (int)status);
        vl53l0x_uart_send_line(msg);
        return;
    }

    snprintf(msg, sizeof(msg),
             "[VL53L0X RANGE] range=%u mm status=%u\r\n",
             (unsigned int)measurement.RangeMilliMeter,
             (unsigned int)measurement.RangeStatus);
    vl53l0x_uart_send_line(msg);
}

static void vl53l0x_acquire_one_sample(void)
{
    VL53L0X_Error status;
    VL53L0X_RangingMeasurementData_t measurement;

    status = vl53l0x_api_init_device();
    if (status != VL53L0X_ERROR_NONE)
    {
        g_raw_tof.timestamp_us = micros();
        g_raw_tof.range_mm = 0;
        g_raw_tof.range_status = (uint8_t)status;
        g_raw_tof.valid = 0;

        preprocess_tof_sample(&g_raw_tof, &g_processed_tof);
        return;
    }

    status = VL53L0X_PerformSingleRangingMeasurement(&g_vl53l0x_dev, &measurement);

    g_raw_tof.timestamp_us = micros();
    g_raw_tof.range_mm = measurement.RangeMilliMeter;
    g_raw_tof.range_status = measurement.RangeStatus;
    g_raw_tof.valid = (status == VL53L0X_ERROR_NONE && measurement.RangeStatus == 0U) ? 1U : 0U;

    preprocess_tof_sample(&g_raw_tof, &g_processed_tof);
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadAttr_t ledHeartbeatAttr = { .name = "ledHeartbeat", .priority = osPriorityLow };
osThreadId_t ledHeartbeatID;

osThreadAttr_t flightControlTaskAttr = { .name = "flightControl", .priority = osPriorityRealtime };
osThreadId_t flightControlTaskID;

osThreadAttr_t imuAcquisitionTaskAttr = { .name = "imuAcquisition", .priority = osPriorityRealtime, .stack_size = 1536 };
osThreadId_t imuAcquisitionTaskID;

osThreadAttr_t uartCommTaskAttr = { .name = "uartCommTask", .priority = osPriorityLow, .stack_size = 1536 };
osThreadId_t uartCommTaskID;

/*osThreadAttr_t tofAcquisitionTaskAttr = {
    .name = "tofAcquisition",
    .priority = osPriorityNormal,
    .stack_size = 1536
};
osThreadId_t tofAcquisitionTaskID;*/

osThreadAttr_t RPYTaskAttr = {.name = "rpyPIDTask", .priority = osPriorityRealtime,
		.stack_size = 1536}; //needed?
osThreadId_t RPYTaskID;

osThreadAttr_t altitudeTaskAttr = {.name = "altitudePIDTask", .priority = osPriorityRealtime,
		.stack_size = 1536}; //needed?
osThreadId_t altitudeTaskID;

const osMutexAttr_t IMUDataMutexAttr = {
  "IMUDataMutex",                          // human readable mutex name
  osMutexRecursive | osMutexPrioInherit,    // attr_bits
  NULL,                                     // memory for control block
  0U                                        // size for control block
};
osMutexId_t IMUDataMutexID;

const osMutexAttr_t altitudeDataMutexAttr = {
  "altitudeDataMutex",                          // human readable mutex name
  osMutexRecursive | osMutexPrioInherit,    // attr_bits
  NULL,                                     // memory for control block
  0U                                        // size for control block
};
osMutexId_t altitudeDataMutexID;

const osMutexAttr_t outputThrustDataMutexAttr = {
  "outputThrustDataMutex",                          // human readable mutex name
  osMutexRecursive | osMutexPrioInherit,    // attr_bits
  NULL,                                     // memory for control block
  0U                                        // size for control block
};
osMutexId_t outputThrustDataMutexID;


osSemaphoreId_t RPYReleaseSemID;
osSemaphoreId_t altitudeReleaseSemID;

osTimerId_t RPYTimer;
static void RPYTimerCallback(void *argument) {
	osSemaphoreRelease(RPYReleaseSemID);
}


osTimerId_t altitudeTimer;
static void altitudeTimerCallback(void *argument) {
	osSemaphoreRelease(altitudeReleaseSemID);
}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RPY_TIME_STEP (2 / 1e3) //2 ms
#define ALT_TIME_STEP (10 / 1e3) //10 ms
#define MAX_OUTPUT 100 //no clue

#define PID_SEMAPHORE_TIMEOUT 25
#define MUTEX_TIMEOUT 25

// Filter coefficient (0.95 to 0.99 is standard)
#define FILTER_ALPHA 0.98f

//tuning: start at 5, then
#define ROLL_MAX_I 10.0f //assuming motor output of 0-100, 10-15
#define PITCH_MAX_I 10.0f
#define YAW_MAX_I 20.0f //20-25
#define ALT_MAX_I 10.0f

#define msToTicks(x) ((x * osKernelGetTickFreq()) / 1000)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct{
	int32_t kp;
	int32_t ki;
	int32_t kd;
	int32_t setpoint; //in struct? cause it will need a mutex around it
} PID_params_t;


typedef struct{
	float roll;
	float pitch;
	float yaw;
	uint32_t timestamp_us;
	bool initialized;
} IMU_vals_t;



static PID_params_t rollPIDParams = {1,1,1,0};
static PID_params_t pitchPIDParams = {1,1,1,0};
static PID_params_t yawPIDParams = {1,1,1,0};
static PID_params_t altitudePIDParams = {1,1,1,0};


static float globalAltitudeOuput; //needed? static
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void applicationInit(void);
void ledHeartbeatTask(void *argument);
void uartCommTask(void *argument);
void IMUAcquisitionTask(void *argument);
void flightControlTask(void *argument);
void RPY_PID_task(void *arguments);
void altitude_PID_task(void *arguments);



/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;
	__disable_irq();
	while (1)
	{
	}
}

static void calibrate_gyro_bias(void)
{
    const uint32_t num_samples = 300;
    uint32_t collected = 0;

    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    LSM6DSR_Axes_t gyro = {0};
    uint8_t gyro_ready = 0;

    /* Small startup settle time */
    HAL_Delay(500);

    while (collected < num_samples)
    {
        LSM6DSR_GYRO_Get_DRDY_Status(&MotionSensor, &gyro_ready);

        if (gyro_ready)
        {
            if (LSM6DSR_GYRO_GetAxes(&MotionSensor, &gyro) == LSM6DSR_OK)
            {
                sum_x += gyro.x;
                sum_y += gyro.y;
                sum_z += gyro.z;
                collected++;
            }
        }

        HAL_Delay(2);
    }

    g_gyro_bias_x_mdps = (float)sum_x / (float)num_samples;
    g_gyro_bias_y_mdps = (float)sum_y / (float)num_samples;
    g_gyro_bias_z_mdps = (float)sum_z / (float)num_samples;
}


static void check_baro_overrun_flags(void)  // to check if all data in barometer is streaming correctly
{
    uint8_t status = 0;

    g_baro_status_checks++;

    if (LPS22HH_Read_Reg(&BaroSensor, 0x27, &status) == LPS22HH_OK)
    {
        if (status & (1U << 5))   /* T_OR */
        {
            g_baro_temp_overrun_count++;
        }

        if (status & (1U << 4))   /* P_OR */
        {
            g_baro_press_overrun_count++;
        }
    }
}

static void check_imu_fifo_status(void)
{
    uint16_t num_samples = 0;
    lsm6dsr_reg_t reg;

    g_imu_fifo_status_checks++;

    if (LSM6DSR_FIFO_Get_Num_Samples(&MotionSensor, &num_samples) == LSM6DSR_OK)
    {
        g_imu_fifo_last_samples = num_samples;

        if (num_samples > g_imu_fifo_max_samples)
        {
            g_imu_fifo_max_samples = num_samples;
        }
    }

    if (lsm6dsr_read_reg(&(MotionSensor.Ctx), LSM6DSR_FIFO_STATUS2, &reg.byte, 1) == LSM6DSR_OK)
    {
        if (reg.fifo_status2.fifo_ovr_ia)
        {
            g_imu_fifo_overrun_count++;
        }
    }
}

static void drain_imu_fifo_once(void)
{
    uint16_t num_samples = 0;
    uint8_t tag = 0;
    uint8_t data[6];
    uint16_t to_drain = 0;
    uint16_t i = 0;

    if (LSM6DSR_FIFO_Get_Num_Samples(&MotionSensor, &num_samples) != LSM6DSR_OK)
    {
        return;
    }

    if (num_samples == 0)
    {
        return;
    }

    /* Drain up to 8 FIFO entries per loop */
    to_drain = (num_samples > 8U) ? 8U : num_samples;

    for (i = 0; i < to_drain; i++)
    {
        if (LSM6DSR_FIFO_Get_Tag(&MotionSensor, &tag) != LSM6DSR_OK)
        {
            break;
        }

        if (LSM6DSR_FIFO_Get_Data(&MotionSensor, data) != LSM6DSR_OK)
        {
            break;
        }

        g_imu_fifo_drained_samples++;
    }
}
/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void applicationInit(void)
{
    assert(MX_LSM6DSR_Init() == LSM6DSR_OK);
    assert(MX_LSM6DSR_FIFO_Test_Init() == LSM6DSR_OK);
    assert(MX_LPS22HH_Init() == LPS22HH_OK);
    calibrate_gyro_bias();
    g_imu_task_loops = 0;
    g_acc_ready_count = 0;
    g_gyro_ready_count = 0;
    g_acc_read_count = 0;
    g_gyro_read_count = 0;

    ledHeartbeatID = osThreadNew(ledHeartbeatTask, NULL, &ledHeartbeatAttr);
    assert(ledHeartbeatID != 0);

//    flightControlTaskID = osThreadNew(flightControlTask, NULL, &flightControlTaskAttr);
//    assert(flightControlTaskID != 0);

    imuAcquisitionTaskID = osThreadNew(IMUAcquisitionTask, NULL, &imuAcquisitionTaskAttr);
    assert(imuAcquisitionTaskID != 0);

//    uartCommTaskID = osThreadNew(uartCommTask, NULL, &uartCommTaskAttr);
//    assert(uartCommTaskID != 0);


    //------
        //set up mutexes and sempahores for PID
    	IMUDataMutexID = osMutexNew(&IMUDataMutexAttr);
    	assert(IMUDataMutexID != NULL);

    	altitudeDataMutexID = osMutexNew(&altitudeDataMutexAttr);
    	assert(altitudeDataMutexID != NULL);

    	outputThrustDataMutexID = osMutexNew(&outputThrustDataMutexAttr);
    	assert(outputThrustDataMutexID != NULL);

    	RPYReleaseSemID = osSemaphoreNew(1, 1, NULL);
    	assert(RPYReleaseSemID != NULL);

    	altitudeReleaseSemID = osSemaphoreNew(1, 1, NULL);
    	assert(altitudeReleaseSemID != NULL);

    	//set up and create PID tasks
    	RPYTaskID = osThreadNew(RPY_PID_task, NULL, &RPYTaskAttr);
    	assert(RPYTaskID != 0);

    	altitudeTaskID = osThreadNew(altitude_PID_task, NULL, &altitudeTaskAttr);
    	assert(altitudeTaskID != 0);

    	//configure PID sequencer timers
    	RPYTimer = osTimerNew(RPYTimerCallback, osTimerPeriodic, NULL, NULL);
    	altitudeTimer = osTimerNew(altitudeTimerCallback, osTimerPeriodic, NULL, NULL);

    	//asserts?
    	//start PID sequencer timers
    	osTimerStart(RPYTimer, msToTicks(2));
    	osTimerStart(altitudeTimer, msToTicks(10));

    	//here?
    	//start PWM control for motors
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


}

//ai provided algorithm
void update_orientation(IMU_vals_t *state, const processed_imu_sample_t *sample) {
    if (!state->initialized) {
        // Initial setup: Use accel for roll/pitch, zero for yaw
        state->roll = atan2f(sample->accel_y_mps2, sample->accel_z_mps2);
        state->pitch = atan2f(-sample->accel_x_mps2,
                             sqrtf(sample->accel_y_mps2 * sample->accel_y_mps2 +
                                   sample->accel_z_mps2 * sample->accel_z_mps2));
        state->yaw = 0.0f;
        state->timestamp_us = sample->timestamp_us;
        state->initialized = true;
        return;
    }

    // 1. Calculate Delta Time (dt)
    float dt = (float)(sample->timestamp_us - state->timestamp_us) / 1000000.0f;
    state->timestamp_us = sample->timestamp_us;

    // 2. Calculate Accel Angles (The "long-term" truth)
    float roll_accel = atan2f(sample->accel_y_mps2, sample->accel_z_mps2);
    float pitch_accel = atan2f(-sample->accel_x_mps2,
                              sqrtf(sample->accel_y_mps2 * sample->accel_y_mps2 +
                                    sample->accel_z_mps2 * sample->accel_z_mps2));

    // 3. Complementary Filter
    // Angle = Alpha * (Angle + Gyro_Step) + (1 - Alpha) * Accel_Angle
    state->roll = FILTER_ALPHA * (state->roll + sample->gyro_x_rps * dt) +
                  (1.0f - FILTER_ALPHA) * roll_accel;

    state->pitch = FILTER_ALPHA * (state->pitch + sample->gyro_y_rps * dt) +
                   (1.0f - FILTER_ALPHA) * pitch_accel;

    // 4. Yaw (Integration only - will drift without a magnetometer)
    state->yaw += sample->gyro_z_rps * dt;
}



void writeToMotors(int motorFR, int motorFL, int motorBR, int motorBL){
	if(motorFR > 100) motorFR = 100;
	if(motorFL > 100) motorFL = 100;
	if(motorBR > 100) motorBR = 100;
	if(motorBL > 100) motorBL = 100;

	//motor vals are between 0-2099, y = 2099/100 * x
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 20.99 * motorFR);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 20.99 * motorFL);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 20.99 * motorBR);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 20.99 * motorBL);
}

void RPY_PID_task(void *arguments){
	//imu read 104 hz

	(void) arguments;

	float rollIntegral = 0;
	float rollLastError = 0;

	float pitchIntegral = 0;
	float pitchLastError = 0;

	float yawIntegral = 0;
	float yawLastError = 0;

	IMU_vals_t currentState = {0};

	uint32_t lastTimestampUs = 0;
	bool isFirstRun = true;


	for(;;){

		//have GPIO pin go high here, then low at the very end of task and measure Ci with scope

		//synchronization for 500 hz (2 ms period) from RTOS timer
		osSemaphoreAcquire(RPYReleaseSemID, PID_SEMAPHORE_TIMEOUT); //determine proper timeout val


		osMutexAcquire(IMUDataMutexID, MUTEX_TIMEOUT); //lock mutex around IMU values, copy, then unlock
		processed_imu_sample_t currentProcessedIMU = g_processed_imu;
		osMutexRelease(IMUDataMutexID);//unlock


		float dt;
		if(isFirstRun){
			dt = 0.002f; // Default for first run (1/500Hz)
			lastTimestampUs = currentProcessedIMU.timestamp_us;
			isFirstRun = false;
		}else {
			dt = (float)(currentProcessedIMU.timestamp_us - lastTimestampUs) / 1000000.0f;
			lastTimestampUs = currentProcessedIMU.timestamp_us;
		}

		if(dt <= 0.0f || dt > 0.05f) dt = 0.002f;

		//process accel and velocity vals into RPY
		update_orientation(&currentState, &currentProcessedIMU);


		//get current roll error
		float rollError = rollPIDParams.setpoint - currentState.roll;
		//get current pitch error
		float pitchError = pitchPIDParams.setpoint - currentState.pitch;
		//get current yaw error
		float yawError = yawPIDParams.setpoint - currentState.yaw;


		//roll intergral and derivative calcs
		rollIntegral += rollError * dt;
		if (rollIntegral > ROLL_MAX_I) rollIntegral = ROLL_MAX_I;
		else if (rollIntegral < -ROLL_MAX_I) rollIntegral = -ROLL_MAX_I;

		float rollDerivative = (rollError - rollLastError) / dt;

		rollLastError = rollError;


		//pitch intergral and derivative calcs
		pitchIntegral += pitchError * dt;
		if (pitchIntegral > PITCH_MAX_I) pitchIntegral = PITCH_MAX_I;
		else if (pitchIntegral < -PITCH_MAX_I) pitchIntegral = -PITCH_MAX_I;

		float pitchDerivative = (pitchError - pitchLastError) / dt;

		pitchLastError = pitchError;


		//yaw intergral and derivative calcs
		yawIntegral += yawError * dt;
		if (yawIntegral > YAW_MAX_I) yawIntegral = YAW_MAX_I;
		else if (yawIntegral < -YAW_MAX_I) yawIntegral = -YAW_MAX_I;

		float yawDerivative = (yawError - yawLastError) / dt;

		yawLastError = yawError;


		//calculate outputs
		float rollOutput = (rollPIDParams.kp * rollError) + (rollPIDParams.ki * rollIntegral) + (rollPIDParams.kd * rollDerivative);
		float pitchOutput = (pitchPIDParams.kp * pitchError) + (pitchPIDParams.ki * pitchIntegral) + (pitchPIDParams.kd * pitchDerivative);
		float yawOutput = (yawPIDParams.kp * yawError) + (yawPIDParams.ki * yawIntegral) + (yawPIDParams.kd * yawDerivative);

		//need to clamp output?
		if(rollOutput > MAX_OUTPUT) rollOutput = MAX_OUTPUT;
		if(pitchOutput > MAX_OUTPUT) pitchOutput = MAX_OUTPUT;
		if(yawOutput > MAX_OUTPUT) yawOutput = MAX_OUTPUT;


		float latestAltitude = globalAltitudeOuput;

		//motor mixing algo (MMA)
		float motorFR = latestAltitude + yawOutput + pitchOutput + rollOutput;
		float motorFL = latestAltitude - yawOutput + pitchOutput - rollOutput;
		float motorBR = latestAltitude - yawOutput - pitchOutput + rollOutput;
		float motorBL = latestAltitude + yawOutput - pitchOutput - rollOutput;

		//cap output at 100% (?)
		if(motorFR > 100) motorFR = 100;
		if(motorFL > 100) motorFL = 100;
		if(motorBR > 100) motorBR = 100;
		if(motorBL > 100) motorBL = 100;

		writeToMotors(motorFR, motorFL, motorBR, motorBL); //replace with real pwm function to change motor speeds
	}
}

void altitude_PID_task(void *arguments){

	(void) arguments;

	float altitudeIntegral;
	float altitudeLastError;
	float altDerivativeFiltered = 0;

	float currentAltitude;

	uint32_t lastBaroTimestampUs = 0;
	bool isFirstRun = true;

	// Filter coefficient for the D-term (0.0 to 1.0)
	// 1.0 = no filter, 0.1 = heavy filtering
	const float d_filter_alpha = 0.2f;

	for(;;){

		//turn led on here, then off at the very end of task and measure Ci with scope

		//barometer can read 100-200 hz, lidar ~30hz
		//synchronization for 100hz release from timer
		osSemaphoreAcquire(altitudeReleaseSemID, PID_SEMAPHORE_TIMEOUT); //timeout val ok?

		//get current altitude
		osMutexAcquire(altitudeDataMutexID, MUTEX_TIMEOUT);
		processed_baro_sample_t currentBarometerReading = g_processed_baro;
		osMutexRelease(altitudeDataMutexID);

		currentAltitude = currentBarometerReading.altitude_m;


		float dt;
		if (isFirstRun) {
			dt = 0.01f; // Default 100Hz
			lastBaroTimestampUs = currentBarometerReading.timestamp_us;
			isFirstRun = false;
		} else {
			dt = (float)(currentBarometerReading.timestamp_us - lastBaroTimestampUs) / 1000000.0f;
			lastBaroTimestampUs = currentBarometerReading.timestamp_us;
		}


		if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;


		float altitudeError = altitudePIDParams.setpoint - currentAltitude;


		altitudeIntegral += altitudeError * dt;
		if (altitudeIntegral > ALT_MAX_I) altitudeIntegral = ALT_MAX_I;
		else if (altitudeIntegral < -ALT_MAX_I) altitudeIntegral = -ALT_MAX_I;

		float rawDerivative = (altitudeError - altitudeLastError) / dt;
		altDerivativeFiltered = (d_filter_alpha * rawDerivative) + ((1.0f - d_filter_alpha) * altDerivativeFiltered);
		altitudeLastError = altitudeError;

//		float altitudeDerivative = (altitudeError - altitudeLastError) / dt;
//		altitudeLastError = altitudeError;

		float altitudeOutput = (altitudePIDParams.kp * altitudeError) + (altitudePIDParams.ki * altitudeIntegral) + (altitudePIDParams.kd * altDerivativeFiltered);

		//clamp output?
		if(altitudeOutput > MAX_OUTPUT) altitudeOutput = MAX_OUTPUT;

		osMutexAcquire(outputThrustDataMutexID, PID_SEMAPHORE_TIMEOUT); //mutex unlock
		globalAltitudeOuput = altitudeOutput;
		osMutexRelease(outputThrustDataMutexID); //mutex unlock
	}
}


void ledHeartbeatTask(void *argument)
{
    (void)argument;
    static bool ledState = false;

    while (1)
    {
        setUserLEDOne(ledState);
        ledState = !ledState;
        osDelay(500);
    }
}

void uartCommTask(void *argument)
{
    (void)argument;
    osDelay(200);

    while (1)
    {
//        vl53l0x_acquire_one_sample();
//        tof_uart_send_processed_line(&g_processed_tof);
        osDelay(200);
    }
}
/*void ToFAcquisitionTask(void *argument)
{
    (void)argument;
    VL53L0X_Error status;

    osDelay(200);

    status = vl53l0x_api_init_device();
    if (status != VL53L0X_ERROR_NONE)
    {
        vl53l0x_uart_send_line("[VL53L0X] init failed\r\n");

        while (1)
        {
            osDelay(1000);
        }
    }

    while (1)
    {
        vl53l0x_acquire_one_sample();

        if (g_processed_tof.valid)
        {
            g_latest_valid_tof_m = g_processed_tof.range_m;
            g_latest_valid_tof_available = 1U;
        }

        osDelay(100);   // 10 Hz, matches your table
    }
}*/

void IMUAcquisitionTask(void *argument)
{
	(void)argument;

	LSM6DSR_Axes_t accel = {0};
	LSM6DSR_Axes_t gyro = {0};
	float pressure_hpa = 0.0f;
	float temperature_c = 0.0f;

	uint8_t acc_ready = 0;
	uint8_t gyro_ready = 0;
	uint8_t press_ready = 0;
	uint8_t temp_ready = 0;

	while (1)
	{
		setUserLEDTwo(0);
		g_imu_task_loops++;  // to count how many times this runs to see if were missing aby samples

		LSM6DSR_ACC_Get_DRDY_Status(&MotionSensor, &acc_ready);
		LSM6DSR_GYRO_Get_DRDY_Status(&MotionSensor, &gyro_ready);
		check_imu_fifo_status();
		drain_imu_fifo_once();

		if (acc_ready)
		{
		    g_acc_ready_count++;
		}

		if (gyro_ready)
		{
		    g_gyro_ready_count++;
		}

		/*if (acc_ready)
		{
			LSM6DSR_ACC_GetAxes(&MotionSensor, &accel);
			g_acc_read_count++;
		}

		if (gyro_ready)
		{
			LSM6DSR_GYRO_GetAxes(&MotionSensor, &gyro);
			 g_gyro_read_count++;
		}*/

		g_raw_imu.timestamp_us = micros();
		g_raw_imu.accel_x_mg = accel.x;
		g_raw_imu.accel_y_mg = accel.y;
		g_raw_imu.accel_z_mg = accel.z;
		g_raw_imu.gyro_x_mdps = gyro.x;
		g_raw_imu.gyro_y_mdps = gyro.y;
		g_raw_imu.gyro_z_mdps = gyro.z;
		g_raw_imu.accel_ready = acc_ready;
		g_raw_imu.gyro_ready = gyro_ready;

		preprocess_imu_sample(&g_raw_imu, &g_processed_imu);

		LPS22HH_PRESS_Get_DRDY_Status(&BaroSensor, &press_ready);
		LPS22HH_TEMP_Get_DRDY_Status(&BaroSensor, &temp_ready);
		check_baro_overrun_flags();

		if (press_ready)
		{
			LPS22HH_PRESS_GetPressure(&BaroSensor, &pressure_hpa);
		}

		if (temp_ready)
		{
			LPS22HH_TEMP_GetTemperature(&BaroSensor, &temperature_c);
		}

		g_raw_baro.timestamp_us = micros();
		g_raw_baro.pressure_hpa = pressure_hpa;
		g_raw_baro.temperature_c = temperature_c;
		g_raw_baro.pressure_ready = press_ready;
		g_raw_baro.temperature_ready = temp_ready;

		preprocess_baro_sample(&g_raw_baro, &g_processed_baro);

		/* Keep UART output light for now */
		//imu_uart_send_raw_line(&accel, &gyro);
		//imu_uart_send_processed_line(&g_processed_imu);
		//baro_uart_send_processed_line(&g_processed_baro);
		//debug_output_send_current_mode(&accel, &gyro, &g_processed_imu, &g_processed_baro);
		//baro_overrun_send_line();
		//imu_fifo_send_line();
		setUserLEDTwo(1);
		osDelay(IMU_TASK_PERIOD_MS);
	}
}

/*void flightControlTask(void *argument)
{
	(void)argument;
	while (1)
	{
		osDelay(100);
	}
}*/

void flightControlTask(void *argument)
{
    (void)argument;

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

    while (1)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

        osDelay(100);
    }
}

/* USER CODE END Application */

