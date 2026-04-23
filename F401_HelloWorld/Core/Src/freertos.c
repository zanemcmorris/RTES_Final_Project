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
#include <assert.h>
#include "sensor_manager.h"

#define USER_LED_1_PORT (GPIOB)
#define USER_LED_2_PORT (GPIOB)
#define USER_LED_1_PIN  (GPIO_PIN_4)
#define USER_LED_2_PIN  (GPIO_PIN_5)

extern TIM_HandleTypeDef htim4;

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
static inline void setUserLEDOne(uint8_t state)
{
	HAL_GPIO_WritePin(USER_LED_1_PORT, USER_LED_1_PIN, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline void setUserLEDTwo(uint8_t state)
{
	HAL_GPIO_WritePin(USER_LED_2_PORT, USER_LED_2_PIN, state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;
	__disable_irq();
	while (1)
	{
	}
}
/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void applicationInit(void)
{
    assert(SensorManager_Init() == 0);

    ledHeartbeatID = osThreadNew(ledHeartbeatTask, NULL, &ledHeartbeatAttr);
    assert(ledHeartbeatID != 0);

    flightControlTaskID = osThreadNew(flightControlTask, NULL, &flightControlTaskAttr);
    assert(flightControlTaskID != 0);

    imuAcquisitionTaskID = osThreadNew(IMUAcquisitionTask, NULL, &imuAcquisitionTaskAttr);
    assert(imuAcquisitionTaskID != 0);

    uartCommTaskID = osThreadNew(uartCommTask, NULL, &uartCommTaskAttr);
    assert(uartCommTaskID != 0);
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
        osDelay(200);
    }
}

/*void ToFAcquisitionTask(void *argument)
{
    (void)argument;
    while (1)
    {
        osDelay(100);
    }
}*/

void IMUAcquisitionTask(void *argument)
{
	(void)argument;

	while (1)
	{
		setUserLEDTwo(0);
		SensorManager_RunOnce();
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
