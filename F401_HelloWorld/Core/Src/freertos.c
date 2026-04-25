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
#include "PID.h"
#include <stdio.h>
#include "app_bluenrg_ms.h"


#define ALTITUDE_OFFSET_M (1)

extern TIM_HandleTypeDef htim4;
extern processed_imu_sample_t g_processed_imu;
extern processed_baro_sample_t g_processed_baro;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadAttr_t ledHeartbeatAttr = { .name = "ledHeartbeat", .priority =
		osPriorityLow };
osThreadId_t ledHeartbeatID;

osThreadAttr_t imuAcquisitionTaskAttr = { .name = "imuAcquisition", .priority =
		osPriorityRealtime, .stack_size = 1536 };
osThreadId_t imuAcquisitionTaskID;

osThreadAttr_t RPYTaskAttr = { .name = "rpyPIDTask", .priority =
		osPriorityRealtime, .stack_size = 1024 }; //needed?
osThreadId_t RPYTaskID;

osThreadAttr_t altitudeTaskAttr = { .name = "altitudePIDTask", .priority =
		osPriorityRealtime, .stack_size = 1024 }; //needed?
osThreadId_t altitudeTaskID;

osThreadAttr_t uartCommTaskAttr = { .name = "uartCommTask", .priority =
		osPriorityLow, .stack_size = 1024 };
osThreadId_t uartCommTaskID;

osThreadAttr_t bleCommTaskAttr = { .name = "bleComm", .priority = osPriorityLow,
		.stack_size = 1536 };
osThreadId_t bleCommTaskID;

const osMutexAttr_t IMUDataMutexAttr = { "IMUDataMutex", // human readable mutex name
		osMutexRecursive | osMutexPrioInherit,    // attr_bits
		NULL,                                     // memory for control block
		0U                                        // size for control block
		};
osMutexId_t IMUDataMutexID;

const osMutexAttr_t altitudeDataMutexAttr = { "altitudeDataMutex", // human readable mutex name
		osMutexRecursive | osMutexPrioInherit,    // attr_bits
		NULL,                                     // memory for control block
		0U                                        // size for control block
		};
osMutexId_t altitudeDataMutexID;

const osMutexAttr_t outputThrustDataMutexAttr = { "outputThrustDataMutex", // human readable mutex name
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

osEventFlagsId_t bleEventFlags;
extern PID_params_t altitudePIDParams;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define msToTicks(x) ((x * osKernelGetTickFreq()) / 1000)
static uint32_t micros(void) {
	return (uint32_t) ((HAL_GetTick() * 1000U)
			+ ((SysTick->LOAD - SysTick->VAL) * 1000U) / (SysTick->LOAD + 1U));
}
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
void RPY_PID_task(void *arguments);
void altitude_PID_task(void *arguments);
void bluetoothControlTask(void *argument);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
static inline void setUserLEDOne(uint8_t state) {
	HAL_GPIO_WritePin(USER_LED_1_PORT, USER_LED_1_PIN,
			state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline void setUserLEDTwo(uint8_t state) {
	HAL_GPIO_WritePin(USER_LED_2_PORT, USER_LED_2_PIN,
			state ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	(void) xTask;
	(void) pcTaskName;
	__disable_irq();
	while (1) {
	}
}
/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Sets up tasks and other RTOS primitives
 */
void applicationInit(void) {

	int status = 0;
	assert(SensorManager_Init() == 0);

	// Primitive creation
	bleEventFlags = osEventFlagsNew(NULL); //Create the event flag for BLE task
	assert(bleEventFlags != NULL);

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

	// Thread creation
	ledHeartbeatID = osThreadNew(ledHeartbeatTask, NULL, &ledHeartbeatAttr);
	assert(ledHeartbeatID != 0);

	imuAcquisitionTaskID = osThreadNew(IMUAcquisitionTask, NULL,
			&imuAcquisitionTaskAttr);
	assert(imuAcquisitionTaskID != 0);

	uartCommTaskID = osThreadNew(uartCommTask, NULL, &uartCommTaskAttr);
	assert(uartCommTaskID != 0);

	bleCommTaskID = osThreadNew(bluetoothControlTask, NULL, &bleCommTaskAttr);
	assert(bleCommTaskID != 0);

	RPYTaskID = osThreadNew(RPY_PID_task, NULL, &RPYTaskAttr);
	assert(RPYTaskID != 0);

	altitudeTaskID = osThreadNew(altitude_PID_task, NULL, &altitudeTaskAttr);
	assert(altitudeTaskID != 0);

	//configure PID sequencer timers
	RPYTimer = osTimerNew(RPYTimerCallback, osTimerPeriodic, NULL, NULL);
	altitudeTimer = osTimerNew(altitudeTimerCallback, osTimerPeriodic, NULL,
	NULL);

	//start PID sequencer timers
	osTimerStart(RPYTimer, msToTicks(2));
	osTimerStart(altitudeTimer, msToTicks(10));

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

}

void RPY_PID_task(void *arguments) {
	//imu read 417 hz

	RPY_PID_State_t rpyPIDState = { .rollIntegral = 0, .rollLastError = 0,
			.pitchIntegral = 0, .pitchLastError = 0, .yawIntegral = 0,
			.yawLastError = 0, .lastTimestampUs = 0, .isFirstRun = true,
			.currentState = { 0 } };

	for (;;) {
		//have GPIO pin go high here, then low at the very end of task and measure Ci with scope
		//synchronization for 500 hz (2 ms period) from RTOS timer
		osSemaphoreAcquire(RPYReleaseSemID, PID_SEMAPHORE_TIMEOUT); //determine proper timeout val
		int start = micros();
		RPY_RunControlLoop(&rpyPIDState);
		int end = micros();
		int diff = end - start;
	}
}

void altitude_PID_task(void *arguments) {

	Altitude_PID_State_t altPIDState = { .altitudeIntegral = 0,
			.altitudeLastError = 0, .altDerivativeFiltered = 0,
			.currentAltitude = 0, .lastBaroTimestampUs = 0, .isFirstRun = true };

	while (altitudePIDParams.setpoint <= ALTITUDE_OFFSET_M) {
		osMutexAcquire(altitudeDataMutexID, osWaitForever);
		altitudePIDParams.setpoint = g_processed_baro.altitude_m
				+ ALTITUDE_OFFSET_M;
		osMutexRelease(altitudeDataMutexID);
	}

	for (;;) {
		//barometer can read 100-200 hz, lidar ~30hz
		//synchronization for 100hz release from timer
		osSemaphoreAcquire(altitudeReleaseSemID, PID_SEMAPHORE_TIMEOUT); //timeout val ok?
		int start = micros();
		Altitude_RunControlLoop(&altPIDState);
		int end = micros();
		volatile int diff = end - start;
	}
}

void ledHeartbeatTask(void *argument) {
	(void) argument;
	static bool ledState = false;

	while (1) {
		setUserLEDOne(ledState);
		ledState = !ledState;
		osDelay(500);
	}
}

void uartCommTask(void *argument) {
	(void) argument;
	osDelay(200);

	while (1) {
		osDelay(200);
	}
}

void IMUAcquisitionTask(void *argument) {
	(void) argument;

	while (1) {

		int start = micros();
		SensorManager_RunOnce();
		int end = micros();
		int diff = end - start;
		osDelay(IMU_SAMPLING_PERIOD_MS);
	}
}

/*
 Approach 1:
 Since we will be asynchronously sending messages instead of checking the BLE message (polling) every 10msec or something using non-blocking stuff
 1) Trigger an interrupt when the message is received (IRQ - hci_tl_lowlevel_isr) (from Laptop -> MCU)
 2) Parse the message : Example : MOVE RIGHT x,y,z? parse the data, pitch - 10 blah blah need to brainstorm this
 3) Do the necessary stuff and Acknowledge back to Laptop (from MCU -> Laptop)?
 4)
 */
/**
 * @brief Adding Primary task for Bluetooth to send recevived data from python to UART 
 */

void bluetoothControlTask(void *argument) {
	printf("Before Init");
	(void) argument;
	MX_BlueNRG_MS_Init();
	// if (BLE_App_Init() != BLE_APP_OK) {
	// 	printf("BLE_App_Init FAILED\r\n");
	//     /* Blink LED fast forever to signal failure */
	//     while (1) {
	//         HAL_GPIO_TogglePin(USER_LED_1_PORT, USER_LED_1_PIN);
	//         osDelay(100);
	//     }
	// }
	// printf("BLE OK — advertising as FCU_NODE\r\n");
	// HAL_GPIO_WritePin(USER_LED_1_PORT, USER_LED_1_PIN, GPIO_PIN_SET);
	printf("BLE middleware init done\r\n");
	uint32_t last_telem_tick = 0;
	uint32_t telem_counter = 0;

	while (1) {

		osEventFlagsWait(bleEventFlags, 0x01, osFlagsWaitAny | osFlagsNoClear,
				10);
		osEventFlagsClear(bleEventFlags, 0x01);
		MX_BlueNRG_MS_Process();   // app layer
	}
}
/* USER CODE END Application */

