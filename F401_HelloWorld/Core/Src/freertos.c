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

#define USER_LED_1_PORT (GPIOB)
#define USER_LED_2_PORT (GPIOB)
#define USER_LED_1_PIN (GPIO_PIN_4)
#define USER_LED_2_PIN (GPIO_PIN_5)
#define IMU_CS_PORT  GPIOA
#define IMU_CS_PIN   GPIO_PIN_8
#define BARO_CS_PORT GPIOC
#define BARO_CS_PIN  GPIO_PIN_13

#define IMU_TASK_PERIOD_MS (50)
#define IMU_TASK_PERIOD_S  (50.0f / 1000.0f)

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

typedef struct
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
} spi_device_t;

static LSM6DSR_Object_t MotionSensor;
static LPS22HH_Object_t BaroSensor;

static spi_device_t imu_dev = {.hspi = &hspi2, .cs_port = IMU_CS_PORT, .cs_pin =
		IMU_CS_PIN};
static spi_device_t baro_dev = {.hspi = &hspi2, .cs_port = BARO_CS_PORT, .cs_pin =
		BARO_CS_PIN};

/* Integrated motion state — written only by IMUAcquisitionTask */
static LSM6DSR_Axes_t accel_vel_absolute;
static LSM6DSR_Axes_t accel_pos_absolute;
static LSM6DSR_Axes_t gyro_absolute;

static inline void setUserLEDOne(uint8_t state)
{
	HAL_GPIO_WritePin(USER_LED_1_PORT, USER_LED_1_PIN, state ? 0 : 1);
}

static inline void setUserLEDTwo(uint8_t state)
{
	HAL_GPIO_WritePin(USER_LED_2_PORT, USER_LED_2_PIN, state ? 0 : 1);
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

static int32_t spi_bus_read_reg(spi_device_t *dev, uint8_t reg, uint8_t *pData,
		uint16_t Length)
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

static int32_t spi_bus_write_reg(spi_device_t *dev, uint8_t reg, uint8_t *pData,
		uint16_t Length)
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

/* IMU-specific wrappers for the LSM6DSR driver */
static int32_t IMU_SPI2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length)
{
	(void) Addr;
	return spi_bus_read_reg(&imu_dev, (uint8_t) Reg, pData, Length);
}

static int32_t IMU_SPI2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length)
{
	(void) Addr;
	return spi_bus_write_reg(&imu_dev, (uint8_t) Reg, pData, Length);
}

/* Barometer-specific wrappers for the LPS22HH driver */
static int32_t BARO_SPI2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length)
{
	(void) Addr;
	return spi_bus_read_reg(&baro_dev, (uint8_t) Reg, pData, Length);
}

static int32_t BARO_SPI2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length)
{
	(void) Addr;
	return spi_bus_write_reg(&baro_dev, (uint8_t) Reg, pData, Length);
}

/**
 * @brief Configure LSM6DSR IMU on SPI2
 */
static int32_t MX_LSM6DSR_Init(void)
{
	LSM6DSR_IO_t io_ctx;
	uint8_t id = 0;
	int32_t fullScale = 2;

	io_ctx.Init = BSP_SPI2_Init;
	io_ctx.DeInit = BSP_SPI2_DeInit;
	io_ctx.BusType = LSM6DSR_SPI_4WIRES_BUS;
	io_ctx.Address = 0;
	io_ctx.WriteReg = IMU_SPI2_WriteReg;
	io_ctx.ReadReg = IMU_SPI2_ReadReg;
	io_ctx.GetTick = HAL_GetTick;

	if (LSM6DSR_RegisterBusIO(&MotionSensor, &io_ctx) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_Init(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_ReadID(&MotionSensor, &id) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	/* Sanity check: expected WHO_AM_I is 0x6B */
	if (id != 0x6B)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_GYRO_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_ACC_SetOutputDataRate(&MotionSensor, 833.0f) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_ACC_SetFullScale(&MotionSensor, fullScale) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_GYRO_SetOutputDataRate(&MotionSensor, 833.0f) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_GYRO_SetFullScale(&MotionSensor, LSM6DSR_250dps) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	return LSM6DSR_OK;
}

/**
 * @brief Setup the LPS22HH Barometer on SPI2
 */
static int32_t MX_LPS22HH_Init(void)
{
	LPS22HH_IO_t io_ctx;
	uint8_t id = 0;

	io_ctx.Init = BSP_SPI2_Init;
	io_ctx.DeInit = BSP_SPI2_DeInit;
	io_ctx.BusType = LPS22HH_SPI_4WIRES_BUS;
	io_ctx.Address = 0;
	io_ctx.WriteReg = BARO_SPI2_WriteReg;
	io_ctx.ReadReg = BARO_SPI2_ReadReg;
	io_ctx.GetTick = HAL_GetTick;
	io_ctx.Delay = HAL_Delay;

	if (LPS22HH_RegisterBusIO(&BaroSensor, &io_ctx) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_Init(&BaroSensor) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_ReadID(&BaroSensor, &id) != LPS22HH_OK)
		return LPS22HH_ERROR;

	/* LPS22HH WHO_AM_I is 0xB3 */
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

/**
 * @brief Send raw IMU axes over UART for Python plotter.
 *        Accel values are converted from mg -> g; gyro from mdps -> dps.
 */
static void imu_uart_send_raw_line(const LSM6DSR_Axes_t *accel_raw,
		const LSM6DSR_Axes_t *gyro_raw)
{
	char buf[128];
	/* Driver outputs mg (accel) and mdps (gyro) — divide by 1000 for g / dps */
	int len = snprintf(buf, sizeof(buf), "IMU,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
			accel_raw->x / 1000.0f, accel_raw->y / 1000.0f, accel_raw->z / 1000.0f,
			gyro_raw->x / 1000.0f, gyro_raw->y / 1000.0f, gyro_raw->z / 1000.0f);

	if (len > 0)
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
}

/**
 * @brief Send integrated velocity (mg*s -> g*s) and gyro angle (mdps*s -> dps*s) over UART.
 */
static void imu_uart_send_integrated_line(const LSM6DSR_Axes_t *accel_integrated,
		const LSM6DSR_Axes_t *gyro_integrated)
{
	char buf[128];
	/* Integrated values carry the same mg / mdps scale from the raw driver values */
	int len = snprintf(buf, sizeof(buf), "INT,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
			accel_integrated->x / 1000.0f, accel_integrated->y / 1000.0f,
			accel_integrated->z / 1000.0f, gyro_integrated->x / 1000.0f,
			gyro_integrated->y / 1000.0f, gyro_integrated->z / 1000.0f);

	if (len > 0)
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
}

/**
 * @brief Send displacement (double-integrated accel, mg*s^2 -> g*s^2) over UART.
 */
static void imu_uart_send_displacement_line(const LSM6DSR_Axes_t *accel_displacement)
{
	char buf[96];
	int len = snprintf(buf, sizeof(buf), "DISP,%.4f,%.4f,%.4f\r\n",
			accel_displacement->x / 1000.0f, accel_displacement->y / 1000.0f,
			accel_displacement->z / 1000.0f);

	if (len > 0)
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
}

/**
 * @brief Send barometer pressure and temperature over UART.
 */
static void baro_uart_send_line(float pressure_hpa, float temperature_c)
{
	char buf[96];
	int len = snprintf(buf, sizeof(buf), "BARO,%.2f,%.2f\r\n", pressure_hpa,
			temperature_c);

	if (len > 0)
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadAttr_t ledHeartbeatAttr = {.name = "ledHeartbeat", .priority = osPriorityLow};
osThreadId_t ledHeartbeatID;

osThreadAttr_t flightControlTaskAttr = {.name = "flightControl", .priority =
		osPriorityRealtime};
osThreadId_t flightControlTaskID;

osThreadAttr_t imuAcquisitionTaskAttr = {.name = "imuAcquisition", .priority =
		osPriorityRealtime, .stack_size = 1536};
osThreadId_t imuAcquisitionTaskID;

osThreadAttr_t uartCommTaskAttr = {.name = "uartCommTask", .priority = osPriorityRealtime,
		.stack_size = 1536};
osThreadId_t uartCommTaskID;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void applicationInit(void);
void ledHeartbeatTask(void *argument);
void uartCommTask(void *argument);
void IMUAcquisitionTask(void *argument);
void flightControlTask(void *argument);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	(void) xTask;
	(void) pcTaskName;
	__disable_irq();
	while (1)
	{
	}
}
/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Sets up tasks and other RTOS primitives
 */
void applicationInit(void)
{
	assert(MX_LSM6DSR_Init() == LSM6DSR_OK);
	assert(MX_LPS22HH_Init() == LPS22HH_OK);

	ledHeartbeatID = osThreadNew(ledHeartbeatTask, NULL, &ledHeartbeatAttr);
	assert(ledHeartbeatID != 0);

	flightControlTaskID = osThreadNew(flightControlTask, NULL, &flightControlTaskAttr);
	assert(flightControlTaskID != 0);

	imuAcquisitionTaskID = osThreadNew(IMUAcquisitionTask, NULL, &imuAcquisitionTaskAttr);
	assert(imuAcquisitionTaskID != 0);

	uartCommTaskID = osThreadNew(uartCommTask, NULL, &uartCommTaskAttr);
	assert(uartCommTaskID != 0);
}

/**
 * @brief Simple task to blink LEDs while the system is active
 */
void ledHeartbeatTask(void *argument)
{
	(void) argument;
	static bool ledState = false;
	while (1)
	{
		setUserLEDOne(ledState);
		ledState = !ledState;
		osDelay(500);
	}
}

/**
 * @brief Placeholder task — will be replaced with PID flight control and renamed
 */
void uartCommTask(void *argument)
{
	(void) argument;
	while (1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
		osDelay(10);
		// Uncomment to vary duty cycle from 100%.
		// Starting at 100% is kinda handy for testing maximum lift ability on the fly.

//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
//		osDelay(10);

	}
}

/**
 * @brief Task for acquiring IMU and barometer data periodically
 */
void IMUAcquisitionTask(void *argument)
{
	(void) argument;

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

		LSM6DSR_ACC_Get_DRDY_Status(&MotionSensor, &acc_ready);
		LSM6DSR_GYRO_Get_DRDY_Status(&MotionSensor, &gyro_ready);

		if (acc_ready)
		{
			LSM6DSR_ACC_GetAxes(&MotionSensor, &accel);

			/* Integrate acceleration (mg) -> velocity (mg*s) */
			accel_vel_absolute.x += accel.x * IMU_TASK_PERIOD_S;
			accel_vel_absolute.y += accel.y * IMU_TASK_PERIOD_S;
			accel_vel_absolute.z += accel.z * IMU_TASK_PERIOD_S;

			/* Integrate velocity -> displacement (mg*s^2) */
			accel_pos_absolute.x += accel_vel_absolute.x * IMU_TASK_PERIOD_S;
			accel_pos_absolute.y += accel_vel_absolute.y * IMU_TASK_PERIOD_S;
			accel_pos_absolute.z += accel_vel_absolute.z * IMU_TASK_PERIOD_S;
		}

		if (gyro_ready)
		{
			LSM6DSR_GYRO_GetAxes(&MotionSensor, &gyro);

			/* Integrate gyro rate (mdps) -> angle (mdps*s) */
			gyro_absolute.x += gyro.x * IMU_TASK_PERIOD_S;
			gyro_absolute.y += gyro.y * IMU_TASK_PERIOD_S;
			gyro_absolute.z += gyro.z * IMU_TASK_PERIOD_S;
		}

		LPS22HH_PRESS_Get_DRDY_Status(&BaroSensor, &press_ready);
		LPS22HH_TEMP_Get_DRDY_Status(&BaroSensor, &temp_ready);

		if (press_ready)
			LPS22HH_PRESS_GetPressure(&BaroSensor, &pressure_hpa);

		if (temp_ready)
			LPS22HH_TEMP_GetTemperature(&BaroSensor, &temperature_c);

		if (press_ready)
			baro_uart_send_line(pressure_hpa, temperature_c);

		imu_uart_send_raw_line(&accel, &gyro);
		imu_uart_send_integrated_line(&accel_vel_absolute, &gyro_absolute);
		imu_uart_send_displacement_line(&accel_pos_absolute);

		setUserLEDTwo(1);
		osDelay(IMU_TASK_PERIOD_MS);
	}
}

/**
 * @brief Primary task for flight control
 */
void flightControlTask(void *argument)
{
	(void) argument;
	while (1)
	{
		osDelay(100);
	}
}

/* USER CODE END Application */
