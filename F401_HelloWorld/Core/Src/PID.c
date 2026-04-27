/*
 * PID.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Dillon Brown
 */

#include "PID.h"
#include "stm32f4xx_hal_tim.h"
//BLE 
volatile motor_outputs_t g_motor_outputs = {0};

// enjoy the terms!
static PID_params_t rollPIDParams = { .70, 0.33, 0.08, 0 };
static PID_params_t pitchPIDParams = { .70, 0.33, 0.08, 0 };
static PID_params_t yawPIDParams = { .1, .05, .01, 0 };
PID_params_t altitudePIDParams = { .1, 0, 0, 0 };

static float globalAltitudeOuput; //needed? static
// Filter coefficient for the D-term (0.0 to 1.0)
// 1.0 = no filter, 0.1 = heavy filtering
const float d_filter_alpha = 0.2f;

extern TIM_HandleTypeDef htim4;
extern osMutexId_t IMUDataMutexID;
extern processed_imu_sample_t g_processed_imu;
extern osMutexId_t altitudeDataMutexID;
extern processed_baro_sample_t g_processed_baro;
extern osMutexId_t outputThrustDataMutexID;

void writeToMotors(motor_outputs_t* motors) {
    if (motors->fr > 100)
        motors->fr = 100;
    if (motors->fl > 100)
        motors->fl = 100;
    if (motors->br > 100)
        motors->br = 100;
    if (motors->bl > 100)
        motors->bl = 100;

    if (motors->fr < 0)
        motors->fr = 0;
    if (motors->fl < 0)
        motors->fl = 0;
    if (motors->br < 0)
        motors->br = 0;
    if (motors->bl < 0)
        motors->bl = 0;

    //motor vals are between 0-2099, y = 2099/100 x
    uint32_t frSetting = 2099 * motors->fr;
    uint32_t flSetting = 2099 * motors->fl;
    uint32_t brSetting = 2099 * motors->br;
    uint32_t blSetting = 2099 * motors->bl;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, blSetting);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, frSetting);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, brSetting);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, flSetting);
}

#define ENABLE_ROLL_CONTROL (0)
#define ENABLE_PITCH_CONTROL (1)
#define ENABLE_YAW_CONTROL (0 )
#define ENABLE_ALT_CONTROL (0)

void RPY_RunControlLoop(RPY_PID_State_t *state) {
	osMutexAcquire(IMUDataMutexID, MUTEX_TIMEOUT); //lock mutex around IMU values, copy, then unlock
	processed_imu_sample_t currentProcessedIMU = g_processed_imu;
	osMutexRelease(IMUDataMutexID); //unlock

	float dt;
	if (state->isFirstRun) {
		dt = 0.002f; // Default for first run (1/500Hz)
		state->lastTimestampUs = currentProcessedIMU.timestamp_us;
		state->isFirstRun = false;
	} else {
		dt = (float) (currentProcessedIMU.timestamp_us - state->lastTimestampUs)
				/ 1000000.0f;
		state->lastTimestampUs = currentProcessedIMU.timestamp_us;
	}

	if (dt <= 0.0f || dt > 0.05f)
		dt = 0.002f;

//	state->currentState.roll = currentProcessedIMU.gyro_x_rad_abs;
//	state->currentState.pitch = currentProcessedIMU.gyro_y_rad_abs;

	state->currentState.roll = currentProcessedIMU.combined_roll;
	state->currentState.pitch = currentProcessedIMU.combined_pitch;
	state->currentState.yaw = currentProcessedIMU.gyro_z_rad_abs;

	//get current roll error
	float rollError = rollPIDParams.setpoint - state->currentState.roll;
	//get current pitch error
	float pitchError = pitchPIDParams.setpoint - state->currentState.pitch;
	//get current yaw error
	float yawError = yawPIDParams.setpoint - state->currentState.yaw;

	//roll intergral and derivative calcs
	state->rollIntegral += rollError * dt;
	if (state->rollIntegral > ROLL_MAX_I)
		state->rollIntegral = ROLL_MAX_I;
	else if (state->rollIntegral < -ROLL_MAX_I)
		state->rollIntegral = -ROLL_MAX_I;

	float rollDerivative = (rollError - state->rollLastError) / dt;

	state->rollLastError = rollError;

	//pitch intergral and derivative calcs
	state->pitchIntegral += pitchError * dt;
	if (state->pitchIntegral > PITCH_MAX_I)
		state->pitchIntegral = PITCH_MAX_I;
	else if (state->pitchIntegral < -PITCH_MAX_I)
		state->pitchIntegral = -PITCH_MAX_I;

	float pitchDerivative = (pitchError - state->pitchLastError) / dt;

	state->pitchLastError = pitchError;

	//yaw intergral and derivative calcs
	state->yawIntegral += yawError * dt;
	if (state->yawIntegral > YAW_MAX_I)
		state->yawIntegral = YAW_MAX_I;
	else if (state->yawIntegral < -YAW_MAX_I)
		state->yawIntegral = -YAW_MAX_I;

	float yawDerivative = (yawError - state->yawLastError) / dt;

	state->yawLastError = yawError;

	//calculate outputs
#if ENABLE_ROLL_CONTROL
	float rollOutput = (rollPIDParams.kp * rollError)
			+ (rollPIDParams.ki * state->rollIntegral)
			+ (rollPIDParams.kd * rollDerivative);
#else
	float rollOutput = 0;
#endif

#if ENABLE_PITCH_CONTROL
	float pitchOutput = (pitchPIDParams.kp * pitchError)
			+ (pitchPIDParams.ki * state->pitchIntegral)
			+ (pitchPIDParams.kd * pitchDerivative);
#else
	float pitchOutput = 0;
#endif

#if ENABLE_YAW_CONTROL
	float yawOutput = (yawPIDParams.kp * yawError)
			+ (yawPIDParams.ki * state->yawIntegral)
			+ (yawPIDParams.kd * yawDerivative);
#else
	float yawOutput = 0;
#endif

	//need to clamp output?
	if (rollOutput > MAX_OUTPUT)
		rollOutput = MAX_OUTPUT;
	if (pitchOutput > MAX_OUTPUT)
		pitchOutput = MAX_OUTPUT;
	if (yawOutput > MAX_OUTPUT)
		yawOutput = MAX_OUTPUT;

#if ENABLE_ALT_CONTROL
	float latestAltitude = globalAltitudeOuput;
#else
	float latestAltitude = 0; // Math says ~70% is hovering
#endif

	//motor mixing algo (MMA)
	// TODO: Add these to BLE telemetry - motor values are in [0,1]
	float motorFR = latestAltitude + yawOutput + pitchOutput + rollOutput;
	float motorFL = latestAltitude - yawOutput + pitchOutput - rollOutput;
	float motorBR = latestAltitude - yawOutput - pitchOutput + rollOutput;
	float motorBL = latestAltitude + yawOutput - pitchOutput - rollOutput;

	//cap output at 100% (?)
	if (motorFR > 100)
		motorFR = 100;
	if (motorFL > 100)
		motorFL = 100;
	if (motorBR > 100)
		motorBR = 100;
	if (motorBL > 100)
		motorBL = 100;

	// Post to global motor data
	osMutexAcquire(outputThrustDataMutexID, MUTEX_TIMEOUT);
	g_motor_outputs.fr = motorFR;
	g_motor_outputs.fl = motorFL;
	g_motor_outputs.br = motorBR;
	g_motor_outputs.bl = motorBL;
	osMutexRelease(outputThrustDataMutexID);
}

void Altitude_RunControlLoop(Altitude_PID_State_t *state) {
	//get current altitude
	osMutexAcquire(altitudeDataMutexID, MUTEX_TIMEOUT);
	processed_baro_sample_t currentBarometerReading = g_processed_baro;
	osMutexRelease(altitudeDataMutexID);

	state->currentAltitude = currentBarometerReading.altitude_m;

	float dt;
	if (state->isFirstRun) {
		dt = 0.01f; // Default 100Hz
		state->lastBaroTimestampUs = currentBarometerReading.timestamp_us;
		state->isFirstRun = false;
	} else {
		dt = (float) (currentBarometerReading.timestamp_us
				- state->lastBaroTimestampUs) / 1000000.0f;
		state->lastBaroTimestampUs = currentBarometerReading.timestamp_us;
	}

	if (dt <= 0.0f || dt > 0.1f)
		dt = 0.01f;

	float altitudeError = altitudePIDParams.setpoint - state->currentAltitude;

	state->altitudeIntegral += altitudeError * dt;
	if (state->altitudeIntegral > ALT_MAX_I)
		state->altitudeIntegral = ALT_MAX_I;
	else if (state->altitudeIntegral < -ALT_MAX_I)
		state->altitudeIntegral = -ALT_MAX_I;

	float rawDerivative = (altitudeError - state->altitudeLastError) / dt;
	state->altDerivativeFiltered = (d_filter_alpha * rawDerivative)
			+ ((1.0f - d_filter_alpha) * state->altDerivativeFiltered);
	state->altitudeLastError = altitudeError;

	float altitudeOutput = (altitudePIDParams.kp * altitudeError)
			+ (altitudePIDParams.ki * state->altitudeIntegral)
			+ (altitudePIDParams.kd * state->altDerivativeFiltered);

	//clamp output?
	if (altitudeOutput > MAX_OUTPUT)
		altitudeOutput = MAX_OUTPUT;

	osMutexAcquire(outputThrustDataMutexID, PID_SEMAPHORE_TIMEOUT); //mutex unlock
	globalAltitudeOuput = altitudeOutput;
	osMutexRelease(outputThrustDataMutexID); //mutex unlock
}

