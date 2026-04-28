#include "main.h"
#include "cmsis_os2.h"
#include "vl53l0x_platform.h"

extern I2C_HandleTypeDef hi2c2;

#define VL53L0X_HAL_TIMEOUT_MS   (100U)

static VL53L0X_Error vl53l0x_hal_status_to_error(HAL_StatusTypeDef status)
{
    switch (status)
    {
        case HAL_OK:
            return VL53L0X_ERROR_NONE;

        case HAL_TIMEOUT:
            return VL53L0X_ERROR_TIME_OUT;

        case HAL_BUSY:
            return VL53L0X_ERROR_CONTROL_INTERFACE;

        case HAL_ERROR:
        default:
            return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hi2c2,
                               Dev->I2cDevAddr,
                               index,
                               I2C_MEMADD_SIZE_8BIT,
                               pdata,
                               (uint16_t)count,
                               VL53L0X_HAL_TIMEOUT_MS);

    return vl53l0x_hal_status_to_error(status);
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c2,
                              Dev->I2cDevAddr,
                              index,
                              I2C_MEMADD_SIZE_8BIT,
                              pdata,
                              (uint16_t)count,
                              VL53L0X_HAL_TIMEOUT_MS);

    return vl53l0x_hal_status_to_error(status);
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buffer[2];

    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0xFFU);

    return VL53L0X_WriteMulti(Dev, index, buffer, 2);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buffer[4];

    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)(data >> 16);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)(data & 0xFFU);

    return VL53L0X_WriteMulti(Dev, index, buffer, 4);
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    VL53L0X_Error status;
    uint8_t buffer[2];

    status = VL53L0X_ReadMulti(Dev, index, buffer, 2);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    *data = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    VL53L0X_Error status;
    uint8_t buffer[4];

    status = VL53L0X_ReadMulti(Dev, index, buffer, 4);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    *data = ((uint32_t)buffer[0] << 24) |
            ((uint32_t)buffer[1] << 16) |
            ((uint32_t)buffer[2] << 8)  |
            (uint32_t)buffer[3];

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error status;
    uint8_t data = 0;

    status = VL53L0X_RdByte(Dev, index, &data);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    data = (uint8_t)((data & AndData) | OrData);

    return VL53L0X_WrByte(Dev, index, data);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    HAL_Delay(5);
    return VL53L0X_ERROR_NONE;
}
