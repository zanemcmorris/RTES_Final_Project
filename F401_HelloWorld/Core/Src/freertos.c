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
