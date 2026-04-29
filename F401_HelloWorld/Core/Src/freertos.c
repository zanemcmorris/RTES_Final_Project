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
#include <stdbool.h>
#include "rtos_flags.h"
#include <VL53L0X_def.h>
#include "hci_tl.h" 

#define HOVER_HEIGHT_M (1)

//extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern processed_imu_sample_t g_processed_imu;
extern processed_baro_sample_t g_processed_baro;
volatile bool armMotors = false;
extern volatile motor_outputs_t g_motor_outputs;
const motor_outputs_t motor_zeros = { 0 };
extern volatile uint32_t g_sensor_runonce_cycles_last;
extern volatile uint32_t g_sensor_runonce_cycles_min;
extern volatile uint32_t g_sensor_runonce_cycles_max;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadAttr_t ledHeartbeatAttr = { .name = "ledHeartbeat", .priority =
		osPriorityLow };
osThreadId_t ledHeartbeatID;

osThreadAttr_t imuAcquisitionTaskAttr = { .name = "imuAcquisition", .priority =
		osPriorityRealtime, .stack_size = 1536 };
osThreadId_t imuAcquisitionTaskID;

osThreadAttr_t tofAcquisitionTaskAttr = { .name = "tofAcquisition", .priority =
		osPriorityRealtime, .stack_size = 1536 };
osThreadId_t tofAcquisitionTaskID;

osThreadAttr_t baroAcqTaskAttr = { .name = "baroAcq", .priority =
		(osPriorityRealtime - 4), .stack_size = 3072};
osThreadId_t baroAcqTaskID;

osThreadAttr_t RPYTaskAttr = { .name = "rpyPIDTask", .priority =
		osPriorityRealtime, .stack_size = 1536}; //needed?
osThreadId_t RPYTaskID;

osThreadAttr_t altitudeTaskAttr = { .name = "altitudePIDTask", .priority =
		osPriorityRealtime, .stack_size = 1536}; //needed?
osThreadId_t altitudeTaskID;

osThreadAttr_t uartCommTaskAttr = { .name = "uartCommTask", .priority =
		osPriorityLow, .stack_size = 1536};
osThreadId_t uartCommTaskID;

osThreadAttr_t bleCommTaskAttr = { .name = "bleComm", .priority = osPriorityLow,
		.stack_size = 2048 };
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

const osMutexAttr_t tofDataMutexAttr = { "tofDataMutex",
osMutexRecursive | osMutexPrioInherit,
NULL, 0U };
osMutexId_t tofDataMutexID;

const osMutexAttr_t outputThrustDataMutexAttr = { "outputThrustDataMutex", // human readable mutex name
		osMutexRecursive | osMutexPrioInherit,    // attr_bits
		NULL,                                     // memory for control block
		0U                                        // size for control block
		};

osMutexId_t outputThrustDataMutexID;
osMutexId_t pidMutex;

osSemaphoreId_t IMUReleaseSemID;
osSemaphoreId_t RPYReleaseSemID;
osSemaphoreId_t altitudeReleaseSemID;
osSemaphoreId_t hciEventSemID;
osTimerId_t RPYTimer;
osTimerId_t altitudeTimer;

static void RPYTimerCallback(void *argument) {
	osSemaphoreRelease(RPYReleaseSemID);
}

static void altitudeTimerCallback(void *argument) {
	osSemaphoreRelease(altitudeReleaseSemID);
}

void TIM2_IMU_PeriodElapsedCallback(void) {
	if (IMUReleaseSemID != NULL) {
		osSemaphoreRelease(IMUReleaseSemID);
	}
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
void TOFAcquisitionTask(void *argument);
void flightControlTask(void *argument);
void BaroAcqTask(void *argument);
void RPY_PID_task(void *arguments);
void altitude_PID_task(void *arguments);
void bluetoothControlTask(void *argument);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
	 task. It is essential that code added to this hook function never attempts
	 to block in any way (for example, call xQueueReceive() with a block time
	 specified, or call vTaskDelay()). If the application makes use of the
	 vTaskDelete() API function (as this demo application does) then it is also
	 important that vApplicationIdleHook() is permitted to return to its calling
	 function, because it is the responsibility of the idle task to clean up
	 memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

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

void PrintFreeRTOSStats(void) {
	char buf[512];

	memset(buf, 0, sizeof(buf));

	snprintf(buf, sizeof(buf), "\r\nTask          Abs Time      %% Time\r\n"
			"--------------------------------------\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 100);

	snprintf(buf, sizeof(buf), "\r\nIMU only cycles:\r\n"
			"  last: %lu cycles = %lu us\r\n"
			"  min : %lu cycles = %lu us\r\n"
			"  max : %lu cycles = %lu us\r\n",
			(unsigned long) g_sensor_imu_cycles_last,
			(unsigned long) (g_sensor_imu_cycles_last / 84U),
			(unsigned long) g_sensor_imu_cycles_min,
			(unsigned long) (g_sensor_imu_cycles_min / 84U),
			(unsigned long) g_sensor_imu_cycles_max,
			(unsigned long) (g_sensor_imu_cycles_max / 84U));
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);

	snprintf(buf, sizeof(buf), "\r\nBarometer only cycles:\r\n"
			"  last: %lu cycles = %lu us\r\n"
			"  min : %lu cycles = %lu us\r\n"
			"  max : %lu cycles = %lu us\r\n",
			(unsigned long) g_sensor_baro_cycles_last,
			(unsigned long) (g_sensor_baro_cycles_last / 84U),
			(unsigned long) g_sensor_baro_cycles_min,
			(unsigned long) (g_sensor_baro_cycles_min / 84U),
			(unsigned long) g_sensor_baro_cycles_max,
			(unsigned long) (g_sensor_baro_cycles_max / 84U));
	HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), HAL_MAX_DELAY);

	memset(buf, 0, sizeof(buf));
	vTaskGetRunTimeStats(buf);

	HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 100);

	/*
	 * Snapshot the volatile timing variables once.
	 * This avoids reading them multiple times while SensorManager_RunOnce()
	 * may be updating them in the IMU task.
	 */
	uint32_t last_cycles = g_sensor_runonce_cycles_last;
	uint32_t min_cycles = g_sensor_runonce_cycles_min;
	uint32_t max_cycles = g_sensor_runonce_cycles_max;

	/*
	 * CPU clock is 84 MHz, so:
	 * 84 cycles = 1 microsecond.
	 */
	uint32_t last_us = last_cycles / 84U;
	uint32_t min_us = min_cycles / 84U;
	uint32_t max_us = max_cycles / 84U;

	memset(buf, 0, sizeof(buf));

	snprintf(buf, sizeof(buf), "\r\nSensorManager_RunOnce:\r\n"
			"cycles: last=%lu, min=%lu, max=%lu\r\n"
			"time:   last=%lu us, min=%lu us, max=%lu us\r\n", last_cycles,
			min_cycles, max_cycles, last_us, min_us, max_us);

	HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 100);
}
/* USER CODE END 4 */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Sets up tasks and other RTOS primitives
 */
void applicationInit(void) {

	assert(SensorManager_Init() == 0);

	// Primitive creation
	bleEventFlags = osEventFlagsNew(NULL);
	assert(bleEventFlags != NULL);

	IMUDataMutexID = osMutexNew(&IMUDataMutexAttr);
	assert(IMUDataMutexID != NULL);

	altitudeDataMutexID = osMutexNew(&altitudeDataMutexAttr);
	assert(altitudeDataMutexID != NULL);

	tofDataMutexID = osMutexNew(&tofDataMutexAttr);
	assert(tofDataMutexID != NULL);

	outputThrustDataMutexID = osMutexNew(&outputThrustDataMutexAttr);
	assert(outputThrustDataMutexID != NULL);

	pidMutex = osMutexNew(NULL);
	assert(pidMutex != NULL);

	IMUReleaseSemID = osSemaphoreNew(1, 0, NULL);
	assert(IMUReleaseSemID != NULL);

	RPYReleaseSemID = osSemaphoreNew(1, 1, NULL);
	assert(RPYReleaseSemID != NULL);

	altitudeReleaseSemID = osSemaphoreNew(1, 1, NULL);
	assert(altitudeReleaseSemID != NULL);

	hciEventSemID = osSemaphoreNew(10, 0, NULL);  
	assert(hciEventSemID != NULL);

	// Thread creation
	ledHeartbeatID = osThreadNew(ledHeartbeatTask, NULL, &ledHeartbeatAttr);
	assert(ledHeartbeatID != 0);

	imuAcquisitionTaskID = osThreadNew(IMUAcquisitionTask, NULL,
			&imuAcquisitionTaskAttr);
	assert(imuAcquisitionTaskID != 0);

	tofAcquisitionTaskID = osThreadNew(TOFAcquisitionTask, NULL,
		    &tofAcquisitionTaskAttr);
	assert(tofAcquisitionTaskID != 0);

	baroAcqTaskID = osThreadNew(BaroAcqTask, NULL, &baroAcqTaskAttr);
	assert(baroAcqTaskID != 0);

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

	// Start HW Timer 4 for motor PWM outputs
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
		osSemaphoreAcquire(RPYReleaseSemID, PID_SEMAPHORE_TIMEOUT); //determine proper timeout val
		int start = micros();

		RPY_RunControlLoop(&rpyPIDState);

		osMutexAcquire(outputThrustDataMutexID, MUTEX_TIMEOUT);
		writeToMotors(&g_motor_outputs);
		osMutexRelease(outputThrustDataMutexID);

		int end = micros();
		int diff = end - start;
		diff -= diff; // remove unused warning

	}
}

void altitude_PID_task(void *arguments) {

	Altitude_PID_State_t altPIDState = { .altitudeIntegral = 0,
			.altitudeLastError = 0, .altDerivativeFiltered = 0,
			.currentAltitude = 0, .lastBaroTimestampUs = 0, .isFirstRun = true };

	while (altitudePIDParams.setpoint <= HOVER_HEIGHT_M) {
		osMutexAcquire(altitudeDataMutexID, osWaitForever);
		altitudePIDParams.setpoint = g_processed_baro.altitude_m
				+ HOVER_HEIGHT_M;
		osMutexRelease(altitudeDataMutexID);
		osDelay(50); // Let other tasks run (baro task) so that we can get past this loop
	}

	for (;;) {
		//barometer can read 100-200 hz, lidar ~30hz
		//synchronization for 100hz release from timer
		osSemaphoreAcquire(altitudeReleaseSemID, osWaitForever);
		int start = micros();
		Altitude_RunControlLoop(&altPIDState);
		int end = micros();
		volatile int diff = end - start;
		diff -= diff; //remove unused warning
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
		osDelay(2000);
		PrintFreeRTOSStats();
	}
}

void IMUAcquisitionTask(void *argument) {
	(void) argument;

	/*
	 * Start TIM2 from inside the task so the FreeRTOS scheduler is already running
	 * before TIM2 begins releasing the IMU semaphore from its interrupt callback.
	 */
	HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
	assert(HAL_TIM_Base_Start_IT(&htim2) == HAL_OK);

	while (1) {
		osSemaphoreAcquire(IMUReleaseSemID, osWaitForever);

		SensorManager_RunIMUOnce();
	}
}

void BaroAcqTask(void *argument) {
	(void) argument;

	while (1) {
		SensorManager_RunBaroOnce();

		osDelay(BARO_SAMPLING_PERIOD_MS);
	}
}

void TOFAcquisitionTask(void *argument) {
	(void) argument;

	while (1) {
		//int start = micros();
		vl53l0x_acquire_one_sample();
		// int end = micros();
		// int diff = end - start;
		osDelay(TOF_SAMPLING_PERIOD_MS);
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

// void bluetoothControlTask(void *argument) {
// 	printf("Before Init");
// 	(void) argument;
// 	MX_BlueNRG_MS_Init();
// 	// if (BLE_App_Init() != BLE_APP_OK) {
// 	// 	printf("BLE_App_Init FAILED\r\n");
// 	//     /* Blink LED fast forever to signal failure */
// 	//     while (1) {
// 	//         HAL_GPIO_TogglePin(USER_LED_1_PORT, USER_LED_1_PIN);
// 	//         osDelay(100);
// 	//     }
// 	// }
// 	// printf("BLE OK — advertising as FCU_NODE\r\n");
// 	// HAL_GPIO_WritePin(USER_LED_1_PORT, USER_LED_1_PIN, GPIO_PIN_SET);
// 	printf("BLE middleware init done\r\n");
// 	uint32_t last_telem_tick = 0;
// 	uint32_t telem_counter = 0;

// 	while (1) {

// 		uint32_t flags = osEventFlagsWait(bleEventFlags, 7, osFlagsWaitAny,
// 				10);

// 		if(flags == osFlagsErrorTimeout){
// 			MX_BlueNRG_MS_Process();   // app layer
// 			continue;
// 		}

// 		if(flags & FLAG_BLE_START){
// 			armMotors = true;
// 		}
// 		if(flags & FLAG_BLE_STOP){
// 			armMotors = false;
// 		}
// 	}
// }

void bluetoothControlTask(void *argument) {
    (void) argument;
    MX_BlueNRG_MS_Init();
    printf("BLE middleware init done\r\n");

    while (1) {
        // Block until EITHER:
        //   - EXTI4 fires (hciEventSemID released from ISR), OR
        //   - 10ms timeout for periodic telemetry
        uint32_t hci_ready = osSemaphoreAcquire(hciEventSemID, 10);

        if (hci_ready == osOK) {
            // Drain ALL pending HCI events atomically.
            // hci_notify_asynch_evt (SPI read) + hci_user_evt_proc (dispatch)
            // both run here in task context — no concurrent queue access.
            while (IsDataAvailable()) {
                hci_notify_asynch_evt(NULL);
            }
        }

        // Always run the HCI event processor and telemetry in task context
        // User_Process_Task();      // replaces User_Process()
        // hci_user_evt_proc();      // now ONLY called here, never from ISR
        // Telemetry_Process();      // your packet A-E send logic
		MX_BlueNRG_MS_Process();

        // Handle BLE command flags (non-blocking check)
        uint32_t flags = osEventFlagsWait(bleEventFlags, 
                                          FLAG_BLE_START | FLAG_BLE_STOP,
                                          osFlagsWaitAny, 0);  // 0 = no wait
        if (!(flags & 0x80000000U)) {
            if (flags & FLAG_BLE_START) armMotors = true;
            if (flags & FLAG_BLE_STOP)  armMotors = false;
        }
    }
}
/* USER CODE END Application */

