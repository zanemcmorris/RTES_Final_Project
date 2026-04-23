#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define IMU_TASK_PERIOD_MS (10)

int32_t SensorManager_Init(void);
void SensorManager_RunOnce(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
