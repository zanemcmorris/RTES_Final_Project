/*
 * PID.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Dillon Brown
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
//#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
//#include <stdbool.h>
#include <assert.h>
#include "sensor_manager.h"
#include "PID.h"

#define MAX_OUTPUT 100 //no clue

#define PID_SEMAPHORE_TIMEOUT 25
#define MUTEX_TIMEOUT 25

// Filter coefficient (0.95 to 0.99 is standard)
#define FILTER_ALPHA 0.98f

//tuning: start at 5, then
#define ROLL_MAX_I 2.0f //assuming motor output of 0-100, 10-15
#define PITCH_MAX_I 2.0f
#define YAW_MAX_I 2.0f //20-25
#define ALT_MAX_I 2.0f

//BLE - motor
typedef struct {
	float fr, fl, br, bl;
} motor_outputs_t;
extern volatile motor_outputs_t g_motor_outputs;

typedef struct {
	float kp;
	float ki;
	float kd;
	float setpoint; //in struct? cause it will need a mutex around it
} PID_params_t;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	uint32_t timestamp_us;
	bool initialized;
} IMU_vals_t;

typedef struct {
	float rollIntegral;
	float rollLastError;
	float pitchIntegral;
	float pitchLastError;
	float yawIntegral;
	float yawLastError;
	uint32_t lastTimestampUs;
	bool isFirstRun;
	IMU_vals_t currentState;
} RPY_PID_State_t;

typedef struct {
	float altitudeIntegral;
	float altitudeLastError;
	float altDerivativeFiltered;
	float currentAltitude;
	uint32_t lastBaroTimestampUs;
	bool isFirstRun;
} Altitude_PID_State_t;

void RPY_RunControlLoop(RPY_PID_State_t *state);
void Altitude_RunControlLoop(Altitude_PID_State_t *state);
void writeToMotors(motor_outputs_t* motors);

#endif /* SRC_PID_H_ */
