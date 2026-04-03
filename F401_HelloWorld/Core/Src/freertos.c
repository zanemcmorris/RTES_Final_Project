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

#define USER_LED_1_PORT (GPIOB)
#define USER_LED_2_PORT (GPIOB)
#define USER_LED_1_PIN (GPIO_PIN_4)
#define USER_LED_2_PIN (GPIO_PIN_5)

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;

static inline void setUserLEDOne(uint8_t state)
{
	HAL_GPIO_WritePin(USER_LED_1_PORT, USER_LED_1_PIN, state ? 0 : 1);
}

#define IMU_CS_PORT GPIOA
#define IMU_CS_PIN  GPIO_PIN_8

static LSM6DSR_Object_t MotionSensor;

static void IMU_CS_Low(void)
{
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
}

static void IMU_CS_High(void)
{
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

static int32_t BSP_SPI2_Init(void)
{
	IMU_CS_High();
	return 0;
}

static int32_t BSP_SPI2_DeInit(void)
{
	IMU_CS_High();
	return 0;
}

static int32_t BSP_GetTick(void)
{
	return (int32_t) HAL_GetTick();
}

static int32_t BSP_SPI2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length)
{
	(void) Addr;

	uint8_t txbuf[32];
	uint8_t rxbuf[32];

	if (Length == 0 || Length > (sizeof(txbuf) - 1))
		return LSM6DSR_ERROR;

	txbuf[0] = (uint8_t) Reg | 0x80;   // read bit only
	memset(&txbuf[1], 0x00, Length);
	memset(rxbuf, 0x00, Length + 1);

	IMU_CS_Low();

	if (HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, Length + 1, 100) != HAL_OK)
	{
		IMU_CS_High();
		return LSM6DSR_ERROR;
	}

	IMU_CS_High();

	memcpy(pData, &rxbuf[1], Length);
	return LSM6DSR_OK;
}

static int32_t BSP_SPI2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length)
{
	(void) Addr;

	uint8_t txbuf[32];

	if (Length == 0 || Length > (sizeof(txbuf) - 1))
		return LSM6DSR_ERROR;

	txbuf[0] = (uint8_t) Reg;
	memcpy(&txbuf[1], pData, Length);

	IMU_CS_Low();

	if (HAL_SPI_Transmit(&hspi2, txbuf, Length + 1, 100) != HAL_OK)
	{
		IMU_CS_High();
		return LSM6DSR_ERROR;
	}

	IMU_CS_High();
	return LSM6DSR_OK;
}

static int32_t MX_LSM6DSR_Init(void)
{
	LSM6DSR_IO_t io_ctx;
	uint8_t id = 0;
	int32_t fullScale = 2;

	io_ctx.Init = BSP_SPI2_Init;
	io_ctx.DeInit = BSP_SPI2_DeInit;
	io_ctx.BusType = LSM6DSR_SPI_4WIRES_BUS;
	io_ctx.Address = 0;
	io_ctx.WriteReg = BSP_SPI2_WriteReg;
	io_ctx.ReadReg = BSP_SPI2_ReadReg;
	io_ctx.GetTick = BSP_GetTick;

	if (LSM6DSR_RegisterBusIO(&MotionSensor, &io_ctx) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_Init(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ReadID(&MotionSensor, &id) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	/* Optional sanity check: expected WHO_AM_I is 0x6B */
	if (id != 0x6B)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_GYRO_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_SetOutputDataRate(&MotionSensor, LSM6DSR_XL_ODR_833Hz) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_GYRO_SetOutputDataRate(&MotionSensor, LSM6DSR_XL_ODR_833Hz) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_SetFullScale(&MotionSensor, fullScale) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_GYRO_SetFullScale(&MotionSensor, LSM6DSR_250dps) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	return LSM6DSR_OK;
}

/**
 * @brief Function for printing formatted data for Python plotter
 */
static void imu_uart_send_line(float ax_g, float ay_g, float az_g, float gx_dps,
		float gy_dps, float gz_dps)
{
	char buf[128];
	int len = snprintf(buf, sizeof(buf), "IMU,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", ax_g,
			ay_g, az_g, gx_dps, gy_dps, gz_dps);

	if (len > 0)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadAttr_t ledHeartbeatAttr = {.name = "ledHeartbeat", .priority = osPriorityLow, };
osThreadId_t ledHeartbeatID;

osThreadAttr_t flightControlTaskAttr =
		{.name = "flightControl", .priority =
		osPriorityRealtime, };
osThreadId_t flightControlTaskID;

osThreadAttr_t imuAcquisitionTaskAttr = {.name = "imuAcquisition", .priority =
		osPriorityRealtime, .stack_size = 1536};
osThreadId_t imuAcquisitionTaskID;


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
void applicationInit();
void ledHeartbeatTask();
void uartCommTask();
void IMUAcquisitionTask();
void flightControlTask();

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/**
 * @brief Sets up tasks and other RTOS primitives
 */
void applicationInit()
{
	int status = 0;
	assert(MX_LSM6DSR_Init() == LSM6DSR_OK);

	ledHeartbeatID = osThreadNew(ledHeartbeatTask, NULL, &ledHeartbeatAttr);
	assert(ledHeartbeatID != 0);

	flightControlTaskID = osThreadNew(flightControlTask, NULL, &flightControlTaskAttr);
	assert(flightControlTaskID != 0);

	imuAcquisitionTaskID = osThreadNew(IMUAcquisitionTask, NULL, &imuAcquisitionTaskAttr);
	assert(imuAcquisitionTaskID != 0);



}
/**
 * @brief Simple task to blink LEDs while the system is active
 */
void ledHeartbeatTask()
{
	static bool ledState = 0;
	while (1)
	{
		setUserLEDOne(ledState);
		ledState = !ledState;
		osDelay(500);
	}
}



void uartCommTask()
{

	while (1)
	{
		osDelay(100);
	}
}


/**
 * @brief Task for acquiring IMU data periodically
 */
void IMUAcquisitionTask()
{

	LSM6DSR_Axes_t accel;
	LSM6DSR_Axes_t gyro;

	uint8_t acc_ready = 0;
	uint8_t gyro_ready = 0;

	while (1)
	{
		LSM6DSR_ACC_Get_DRDY_Status(&MotionSensor, &acc_ready);
		LSM6DSR_GYRO_Get_DRDY_Status(&MotionSensor, &gyro_ready);

		if (acc_ready)
		{
			LSM6DSR_ACC_GetAxes(&MotionSensor, &accel);
		}

		if (gyro_ready)
		{
			LSM6DSR_GYRO_GetAxes(&MotionSensor, &gyro);
		}

		imu_uart_send_line(accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

		osDelay(50);

	}

}

/**
 * @brief Primary task for flight control
 */
void flightControlTask()
{
	while (1)
	{
		osDelay(100);
	}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void) xTask;
	(void) pcTaskName;
	__disable_irq();
	while (1)
	{
	}
}
/* USER CODE END Application */

