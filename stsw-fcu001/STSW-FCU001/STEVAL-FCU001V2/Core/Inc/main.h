/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "custom_motion_sensors.h"
#include "custom_motion_sensors_ex.h"
#include "custom_env_sensors.h"
#include "lsm6dsr.h"
#include "lsm6dsr_reg.h"
#include "lps22hh.h"
#include "quaternion.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/**
 * @brief  Sensor axes data structure definition
 */
typedef struct
{
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
} SensorAxesRaw_t;


typedef struct {
    int32_t AXIS_X;
    int32_t AXIS_Y;
    int32_t AXIS_Z;
} AxesRaw_TypeDef;

typedef struct {
    float AXIS_X;
    float AXIS_Y;
    float AXIS_Z;
} AxesRaw_TypeDef_Float;

typedef enum
{
  LED1 = 0,
  LED2 = 1
}Led_TypeDef;

/**
 * @brief  Component's Status enumerator definition.
 */
typedef enum
{
  COMPONENT_OK = 0,
  COMPONENT_ERROR,
  COMPONENT_TIMEOUT,
  COMPONENT_NOT_IMPLEMENTED
} DrvStatusTypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* Update 09/12/16 - Resistors partition for battery voltage monitoring */
#define BAT_RUP 10      /* Pull-up resistor value [Kohm] */
#define BAT_RDW 20      /* Pull-Down resistor value [Kohm] */
//#define MOTENV_DEBUG_CONNECTION 1
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void	BSP_LED_Init(Led_TypeDef Led);
void	BSP_LED_DeInit(Led_TypeDef Led);
void	BSP_LED_On(Led_TypeDef Led);
void	BSP_LED_Off(Led_TypeDef Led);
void 	BSP_LED_Toggle(Led_TypeDef Led);


/** @defgroup BLE default values
* @{
*/
//#define NAME_BLUEMS 'D','R','N','1','1','2','0'
//#define STM32_UUID ((uint32_t *)0x1FFF7A10)
//#define GAP_PERIPHERAL_ROLE_IDB05A1			(0x01)
//#define GAP_BROADCASTER_ROLE_IDB05A1		        (0x02)
//#define GAP_CENTRAL_ROLE_IDB05A1			(0x04)
//#define GAP_OBSERVER_ROLE_IDB05A1			(0x08)
//#define MAC_BLUEMS 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

/*#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
{\
  uuid_struct[0 ] = uuid_0 ; uuid_struct[1 ] = uuid_1 ; uuid_struct[2 ] = uuid_2 ; uuid_struct[3 ] = uuid_3 ; \
  uuid_struct[4 ] = uuid_4 ; uuid_struct[5 ] = uuid_5 ; uuid_struct[6 ] = uuid_6 ; uuid_struct[7 ] = uuid_7 ; \
  uuid_struct[8 ] = uuid_8 ; uuid_struct[9 ] = uuid_9 ; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
  uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}*/


/* Console Service */
//#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
//#define COPY_TERM_CHAR_UUID(uuid_struct)         COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
//#define COPY_STDERR_CHAR_UUID(uuid_struct)       COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Configuration Service */
//#define COPY_CONFIG_SERVICE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
//#define COPY_CONFIG_W2ST_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Define the Max dimesion of the Bluetooth characteristics
for each packet used for Console Service */
//#define W2ST_CONSOLE_MAX_CHAR_LEN 20

/**
 * @name Configuration values.
 * See @ref aci_hal_write_config_data().
 * @{
 */
//#define CONFIG_DATA_PUBADDR_OFFSET          (0x00) /**< Bluetooth public address */
//#define CONFIG_DATA_DIV_OFFSET              (0x06) /**< DIV used to derive CSRK */
//#define CONFIG_DATA_ER_OFFSET               (0x08) /**< Encryption root key used to derive LTK and CSRK */
//#define CONFIG_DATA_IR_OFFSET               (0x18) /**< Identity root key used to derive LTK and CSRK */
//#define CONFIG_DATA_LL_WITHOUT_HOST         (0x2C) /**< Switch on/off Link Layer only mode. Set to 1 to disable Host.
// 	 	 	 	 	 	 	 	 	 	 	 	 	 It can be written only if aci_hal_write_config_data() is the first command 	 	 	 	 	 	 	 	 	 	 	 	 after reset. */
//#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */
/**
 * @name Length for configuration values.
 * See @ref aci_hal_write_config_data().
 * @{
 */
//#define CONFIG_DATA_PUBADDR_LEN             (6)
//#define CONFIG_DATA_DIV_LEN                 (2)
//#define CONFIG_DATA_ER_LEN                  (16)
//#define CONFIG_DATA_IR_LEN                  (16)
//#define CONFIG_DATA_LL_WITHOUT_HOST_LEN     (1)
//#define CONFIG_DATA_MODE_LEN                (1)
//#define CONFIG_DATA_WATCHDOG_DISABLE_LEN    (1)

/**
 * Select the BlueNRG mode configurations.\n
 * @li Mode 1: slave or master, 1 connection, RAM1 only (small GATT DB)
 * @li Mode 2: slave or master, 1 connection, RAM1 and RAM2 (large GATT DB)
 * @li Mode 3: master/slave, 8 connections, RAM1 and RAM2.
 * @li Mode 4: master/slave, 4 connections, RAM1 and RAM2 simultaneous scanning and advertising.
 */
//#define CONFIG_DATA_MODE_OFFSET 			(0x2D)

//#define CONFIG_DATA_WATCHDOG_DISABLE 		(0x2F) /**< Set to 1 to disable watchdog. It is enabled by default. */

/**
 * @anchor Auth_req
 * @name Authentication requirements
 * @{
 */
//#define BONDING				            (0x01)
//#define NO_BONDING				        (0x00)
/**
 * @}
 */

/**
 * @anchor MITM_req
 * @name MITM protection requirements
 * @{
 */
//#define MITM_PROTECTION_NOT_REQUIRED	(0x00)
//#define MITM_PROTECTION_REQUIRED        (0x01)
/**
 * @}
 */

/**
 * @anchor OOB_Data
 * @name Out-Of-Band data
 * @{
 */
//#define OOB_AUTH_DATA_ABSENT		    (0x00)
//#define OOB_AUTH_DATA_PRESENT      		(0x01)
/**
 * @}
 */
/**
 * @anchor Use_fixed_pin
 * @name Use fixed pin
 * @{
 */
//#define USE_FIXED_PIN_FOR_PAIRING		(0x00)
//#define DONOT_USE_FIXED_PIN_FOR_PAIRING	(0x01)
/**
 * @}
 */


//static int HCI_ProcessEvent=0;
//volatile uint32_t HCI_ProcessEvent=0;
extern volatile uint32_t HCI_ProcessEvent;
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};


extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LPS22HH_CS_Pin GPIO_PIN_13
#define LPS22HH_CS_GPIO_Port GPIOC
#define USB_Monitor_Pin GPIO_PIN_14
#define USB_Monitor_GPIO_Port GPIOC
#define BLE_IRQ_Pin GPIO_PIN_4
#define BLE_IRQ_GPIO_Port GPIOA
#define BLE_IRQ_EXTI_IRQn EXTI4_IRQn
#define BLE_CS_Pin GPIO_PIN_0
#define BLE_CS_GPIO_Port GPIOB
#define VBAT_SENSE_Pin GPIO_PIN_1
#define VBAT_SENSE_GPIO_Port GPIOB
#define BLE_RSTN_Pin GPIO_PIN_2
#define BLE_RSTN_GPIO_Port GPIOB
#define LSM6DSR_CS_Pin GPIO_PIN_8
#define LSM6DSR_CS_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LEDn                             2

#define LED1_PIN                         LED1_Pin
#define LED1_GPIO_PORT                   LED1_GPIO_Port
#define LED1_GPIO_CLK_ENABLE()           __GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __GPIOB_CLK_DISABLE()

#define LED2_PIN                       LED2_Pin
#define LED2_GPIO_PORT                 LED2_GPIO_Port
#define LED2_GPIO_CLK_ENABLE()         __GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()        __GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)  do{if((__INDEX__) == 0) LED1_GPIO_CLK_ENABLE(); \
                                            if((__INDEX__) == 1) LED2_GPIO_CLK_ENABLE(); \
                                            }while(0)

#define LEDx_GPIO_CLK_DISABLE(__INDEX__) do{if((__INDEX__) == 0) LED1_GPIO_CLK_DISABLE(); \
                                            if((__INDEX__) == 1) LED2_GPIO_CLK_DISABLE(); \
                                            }while(0)



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
