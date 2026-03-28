/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motor.h"
#include "custom_bus.h"

#include "debug.h"
#include "quaternion.h"
#include "ahrs.h"
#include "flight_control.h"
#include "timer.h"
#include "rc.h"
#include "config_drone.h"

// BLE files
#include "SPBTLE_RF.h"
#include "bluenrg_gatt_server.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "TargetFeatures.h"
#include "custom.h"
#include "bluenrg_l2cap_aci.h"
#include "hci.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t HCI_ProcessEvent=0;
uint8_t joydata[8] = {0,0,0,0,0,0,0,0};

uint32_t uhCCR4_Val = 500;
uint32_t uhCCR1_Val = 5000;

LPS22HH_Object_t LPS22HH_obj;
LSM6DSR_Object_t LSM6DSR_obj;

extern volatile tUserTimer tim;
extern char rc_connection_flag;
extern int16_t gAIL, gELE, gTHR, gRUD;
int16_t gJoystick_status;
int32_t rc_cal_flag = 0;
int32_t rc_enable_motor = 0;
int32_t rc_cal_cnt = 0;
int32_t fly_ready = 0;
unsigned char ch, ch_flag;

uint32_t tim9_event_flag = 0, tim9_cnt = 0, tim9_cnt2 = 0;
float tmp_euler_z = 0;

/* BLE module */
DrvStatusTypeDef testStatus = COMPONENT_OK;
uint8_t test_res_global = 0;
uint8_t testEvent = 0;
uint8_t bdaddr[6];
extern int connected;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

void BlueNRG_Init(void);
static void Init_BlueNRG_Custom_Services(void);
static void SendMotionData(void);
static void SendBattEnvData(void);
static void SendArmingData(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
P_PI_PIDControlTypeDef pid;
EulerAngleTypeDef euler_rc, euler_ahrs, euler_rc_fil, euler_rc_y_pre[4], euler_rc_x_pre[4];
AxesRaw_TypeDef acc, gyro, mag, acc_fil_int, gyro_fil_int, mag_fil_int;
AxesRaw_TypeDef_Float acc_fil, acc_y_pre[4], acc_x_pre[4], acc_ahrs_FIFO[FIFO_Order], acc_FIFO[FIFO_Order], acc_ahrs;
AxesRaw_TypeDef_Float gyro_fil, gyro_y_pre[4], gyro_x_pre[4], gyro_ahrs_FIFO[FIFO_Order], gyro_FIFO[FIFO_Order], gyro_ahrs;
AxesRaw_TypeDef_Float mag_fil;
AxesRaw_TypeDef acc_off_calc, gyro_off_calc, acc_offset, gyro_offset;
EulerAngleTypeDef euler_ahrs_offset;
int sensor_init_cali = 0, sensor_init_cali_count = 0;
int gyro_cali_count = 0;

typedef struct
{
  int16_t X_Degree;
  int16_t Y_Degree;
  int16_t Z_Degree;
}Attitude_Degree;

typedef struct
{
  float a1, a2, b0, b1, b2;
}IIR_Coeff;


//sensor filter
//7hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.922286512869545,  -0.92519529534950118, 0.00072719561998898304, 0.0014543912399779661, 0.00072719561998898304};

//15hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.8337326589246479,  -0.84653197479202391, 0.003199828966843966, 0.0063996579336879321, 0.003199828966843966};

//30hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.66920314293119312,  -0.71663387350415764, 0.011857682643241156, 0.023715365286482312, 0.011857682643241156};

//60hz, 800hz
//IIR_Coeff gyro_fil_coeff = {1.3489677452527946 ,  -0.51398189421967566, 0.041253537241720303, 0.082507074483440607, 0.041253537241720303};

//100hz, 800hz
IIR_Coeff gyro_fil_coeff = {0.94280904158206336,  -0.33333333333333343, 0.09763107293781749 , 0.19526214587563498 , 0.09763107293781749 };

Attitude_Degree  Fly, Fly_offset, Fly_origin;
Gyro_Rad gyro_rad, gyro_degree, gyro_cali_degree;
MotorControlTypeDef motor_pwm;
int count1 = 0, count2 = 0;
AHRS_State_TypeDef ahrs;
float press, press_zero_level;
float temperature;

uint32_t VBAT_Sense;
float VBAT = 0;

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
const uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};


/* BLE */
extern uint8_t set_connectable;
uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
uint32_t ConnectionBleStatus=0;
uint8_t BufferToWrite[256];
int32_t BytesToWrite;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int16_t pid_interval, i;

	  int mytimcnt = 0;
	  acc_fil.AXIS_X = 0;
	  acc_fil.AXIS_Y = 0;
	  acc_fil.AXIS_Z = 0;
	  mag_fil.AXIS_X = 0;
	  mag_fil.AXIS_Y = 0;
	  mag_fil.AXIS_Z = 0;
	  gyro_fil.AXIS_X = 0;
	  gyro_fil.AXIS_Y = 0;
	  gyro_fil.AXIS_Z = 0;
	  euler_rc_fil.thx = 0;
	  euler_rc_fil.thy = 0;
	  euler_rc_fil.thz = 0;
	  acc_off_calc.AXIS_X = 0;
	  acc_off_calc.AXIS_Y = 0;
	  acc_off_calc.AXIS_Z = 0;
	  gyro_off_calc.AXIS_X = 0;
	  gyro_off_calc.AXIS_Y = 0;
	  gyro_off_calc.AXIS_Z = 0;
	  acc_offset.AXIS_X = 0;
	  acc_offset.AXIS_Y = 0;
	  acc_offset.AXIS_Z = 1000;
	  gyro_offset.AXIS_X = 0;
	  gyro_offset.AXIS_Y = 0;
	  gyro_offset.AXIS_Z = 0;
	  euler_rc.thz = euler_ahrs.thz;
	  euler_ahrs_offset.thx = 0;
	  euler_ahrs_offset.thy = 0;

	  for(i=0;i<4;i++)
	  {
	    acc_y_pre[i].AXIS_X = 0;
	    acc_y_pre[i].AXIS_Y = 0;
	    acc_y_pre[i].AXIS_Z = 0;
	    acc_x_pre[i].AXIS_X = 0;
	    acc_x_pre[i].AXIS_Y = 0;
	    acc_x_pre[i].AXIS_Z = 0;
	    gyro_y_pre[i].AXIS_X = 0;
	    gyro_y_pre[i].AXIS_Y = 0;
	    gyro_y_pre[i].AXIS_Z = 0;
	    gyro_x_pre[i].AXIS_X = 0;
	    gyro_x_pre[i].AXIS_Y = 0;
	    gyro_x_pre[i].AXIS_Z = 0;
	    euler_rc_y_pre[i].thx = 0;
	    euler_rc_y_pre[i].thy = 0;
	    euler_rc_y_pre[i].thz = 0;
	    euler_rc_x_pre[i].thx = 0;
	    euler_rc_x_pre[i].thy = 0;
	    euler_rc_x_pre[i].thz = 0;
	  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if (BSP_SPI2_Init()!=BSP_ERROR_NONE)
 	  Error_Handler();

  // if (HCI_TL_SPI_Init(NULL)!=BSP_ERROR_NONE)
  // 	  Error_Handler();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */



  PRINTF("STEVAL-FCU001V2 FW rev.1.0 - May 2021\r\n\r\n");

 //  Initialize Onboard LED
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);

  // initialize All Sensors
  CUSTOM_MOTION_SENSOR_Init(CUSTOM_LSM6DSR_0, MOTION_GYRO | MOTION_ACCELERO);
  CUSTOM_ENV_SENSOR_Init(CUSTOM_LPS22HH_0,ENV_PRESSURE | ENV_TEMPERATURE);

  // Enable All Sensors
  CUSTOM_MOTION_SENSOR_Enable(CUSTOM_LSM6DSR_0, MOTION_GYRO | MOTION_ACCELERO);
  CUSTOM_ENV_SENSOR_Enable(CUSTOM_LPS22HH_0,ENV_PRESSURE | ENV_TEMPERATURE);

  /* Initialize settings for 6-axis MEMS Accelerometer */
   /* ODR 6.6kHz */
   /* FS 4g */
  CUSTOM_MOTION_SENSOR_SetOutputDataRate(CUSTOM_LSM6DSR_0,MOTION_ACCELERO,6660.0f);
  CUSTOM_MOTION_SENSOR_SetFullScale(CUSTOM_LSM6DSR_0,MOTION_ACCELERO,4);

   /* Analog Filter Bandwith @ 1500Hz */
   /* ODR/2 low pass filtered sent to composite filter */
   /* Low pass filter enabled @ ODR/400 */
  // LSM6DSL_ACC_GYRO_IN_ODR_DIV_2      = 0x00,
  // LSM6DSL_ACC_GYRO_LPF2_XL_ENABLE      = 0x80,
  // LSM6DSL_ACC_GYRO_HPCF_XL_DIV400      = 0x60
  CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LSM6DSR_0, LSM6DSR_CTRL8_XL, 0x80 | 0x60 | 0x00);

  /*
  uint8_t tmp_6axis_reg_value;
  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSR_0,0x10, &tmp_6axis_reg_value);
  //tmp_6axis_reg_value = tmp_6axis_reg_value | 0x01;                             // Set LSB to 1 >> Analog filter 400Hz
  tmp_6axis_reg_value = tmp_6axis_reg_value & 0xFE;                              //Set LSB to 0 >> Analog filter 1500Hz
  CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LSM6DSR_0,0x10, (uint8_t) tmp_6axis_reg_value);
  */

  /* Initialize settings for 6-axis MEMS Gyroscope */
  /* Gyroscope settings: full scale 2000dps, ODR 416Hz */
  CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LSM6DSR_0,LSM6DSR_CTRL2_G, 0x6C);

  /* LPF1 FTYPE set to 10b */
  uint8_t tmp_LPF1;
  CUSTOM_MOTION_SENSOR_Read_Register(CUSTOM_LSM6DSR_0,LSM6DSR_CTRL6_C, &tmp_LPF1);
  CUSTOM_MOTION_SENSOR_Write_Register(CUSTOM_LSM6DSR_0,LSM6DSR_CTRL6_C, tmp_LPF1 & 0xFA);

  /* Initialize Remote control*/
  init_remote_control();

  /* Initialize TIM2 for External Remocon RF receiver PWM Input*/
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);

  /* Initialize TIM4 for Motors PWM Output*/
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  /* Initialize General purpose TIM9 50Hz*/
  HAL_TIM_Base_Start_IT(&htim9);

  /* Initialize PID and set Motor PWM to zero */
  PIDControlInit(&pid);
  set_motor_pwm_zero(&motor_pwm);

  /* Setup a timer with 1ms interval */
  pid_interval = (int16_t)(PID_SAMPLING_TIME*1000.0f);
  SetupTimer(&tim, pid_interval);

  /* Start timer */
  StartTimer(&tim);
  ch = 0;
  ch_flag = 0;


  /* BLE communication */
  PRINTF("BLE communication initialization...\n\n");
  BlueNRG_Init();
  /* Initialize the BlueNRG Custom services */
  PRINTF("BLE services initialization...\r\n");
  Init_BlueNRG_Custom_Services();

  CUSTOM_ENV_SENSOR_GetValue(CUSTOM_LPS22HH_0,ENV_PRESSURE,&press_zero_level);
  CUSTOM_ENV_SENSOR_GetValue(CUSTOM_LPS22HH_0,ENV_TEMPERATURE,&temperature);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (HCI_ProcessEvent) {
			HCI_ProcessEvent = 0;
			HCI_Process();
		}

		if (set_connectable) {
			 //Now update the BLE advertize data and make the Board connectable
			setConnectable();
			set_connectable = FALSE;
		}

		if (tim9_event_flag == 1) {     // Timer9 event: frequency 800Hz
			tim9_event_flag = 0;

			count1++;

			acc_ahrs.AXIS_X = 0;
			acc_ahrs.AXIS_Y = 0;
			acc_ahrs.AXIS_Z = 0;
			gyro_ahrs.AXIS_X = 0;
			gyro_ahrs.AXIS_Y = 0;
			gyro_ahrs.AXIS_Z = 0;

			for (i = 0; i < FIFO_Order; i++) {
				acc_ahrs.AXIS_X += acc_ahrs_FIFO[i].AXIS_X;
				acc_ahrs.AXIS_Y += acc_ahrs_FIFO[i].AXIS_Y;
				acc_ahrs.AXIS_Z += acc_ahrs_FIFO[i].AXIS_Z;
				gyro_ahrs.AXIS_X += gyro_ahrs_FIFO[i].AXIS_X;
				gyro_ahrs.AXIS_Y += gyro_ahrs_FIFO[i].AXIS_Y;
				gyro_ahrs.AXIS_Z += gyro_ahrs_FIFO[i].AXIS_Z;
			}

			acc_ahrs.AXIS_X *= FIFO_Order_Recip;
			acc_ahrs.AXIS_Y *= FIFO_Order_Recip;
			acc_ahrs.AXIS_Z *= FIFO_Order_Recip;
			gyro_ahrs.AXIS_X *= FIFO_Order_Recip;
			gyro_ahrs.AXIS_Y *= FIFO_Order_Recip;
			gyro_ahrs.AXIS_Z *= FIFO_Order_Recip;

			acc_fil_int.AXIS_X = (int32_t) acc_ahrs.AXIS_X;
			acc_fil_int.AXIS_Y = (int32_t) acc_ahrs.AXIS_Y;
			acc_fil_int.AXIS_Z = (int32_t) acc_ahrs.AXIS_Z;
			gyro_fil_int.AXIS_X = (int32_t) gyro_ahrs.AXIS_X;
			gyro_fil_int.AXIS_Y = (int32_t) gyro_ahrs.AXIS_Y;
			gyro_fil_int.AXIS_Z = (int32_t) gyro_ahrs.AXIS_Z;

			//PRINTF("%f %f %f %f\n", acc_ahrs.AXIS_X, acc_ahrs.AXIS_Y, gyro_ahrs.AXIS_X, gyro_ahrs.AXIS_Y);

			// AHRS update, quaternion & true gyro data are stored in ahrs
			ahrs_fusion_ag(&acc_ahrs, &gyro_ahrs, &ahrs);

			// Calculate euler angle drone
			QuaternionToEuler(&ahrs.q, &euler_ahrs);

			//BSP_LED_Toggle(LED1);

#ifdef REMOCON_BLE

			//          gRUD = (joydata[3]-128)*(-13);
			//          gTHR = joydata[4]*13;
			//          gAIL = (joydata[5]-128)*(-13);
			//          gELE = (joydata[6]-128)*13;
			gRUD = (joydata[2] - 128) * (-13);
			gTHR = joydata[3] * 13;
			gAIL = (joydata[4] - 128) * (-13);
			gELE = (joydata[5] - 128) * 13;

			/* joydata[6]: seek bar data*/
			/* joydata[7]: additional button data
			 first bit: Takeoff (0 = Land,  1 = Takeoff)
			 second bit: Calibration When it changes status is active
			 third bit: Arming (0 = Disarmed,  1 = Armed) */
			gJoystick_status = joydata[7];
			if ((gJoystick_status & 0x04) == 0x04) {
				rc_enable_motor = 1;
				fly_ready = 1;
				BSP_LED_On(LED2);
			} else {
				rc_enable_motor = 0;
				fly_ready = 0;
			}

			if (connected) {
				rc_connection_flag = 1; /* BLE Remocon connected flag for enabling motor output */
				SendMotionData();
				SendBattEnvData();
				SendArmingData();
			} else {
				rc_connection_flag = 0;
				gTHR = 0;
				rc_enable_motor = 0;
				fly_ready = 0;
				BSP_LED_Off(LED1);
				BSP_LED_Off(LED2);
			}

			if (joydata[7] & 0x02) {
				rc_cal_flag = 1;
				BSP_LED_On(LED1);
			}

#endif

#ifdef REMOCON_PWM
	            if ( (gTHR == 0) && (gELE < - RC_CAL_THRESHOLD) && (gAIL > RC_CAL_THRESHOLD) && (gRUD < - RC_CAL_THRESHOLD))
	            {
	              rc_cal_flag = 1;
	              BSP_LED_On(LED1);
	            }


	            if ( (gTHR == 0) && (gELE < - RC_CAL_THRESHOLD) && (gAIL < - RC_CAL_THRESHOLD) && (gRUD > RC_CAL_THRESHOLD))
	            {
	              rc_enable_motor = 1;
	              fly_ready = 1;
	              BSP_LED_On(LED2);
	            }
#endif

			// Get target euler angle from remote control
			GetTargetEulerAngle(&euler_rc, &euler_ahrs);

			if (gTHR < MIN_THR) {
				euler_ahrs_offset.thx = 0;
				euler_ahrs_offset.thy = 0;
			}

			Fly_origin.X_Degree = (int16_t) (euler_ahrs.thx * 5730);
			Fly_origin.Y_Degree = (int16_t) (euler_ahrs.thy * 5730);
			Fly_origin.Z_Degree = (int16_t) (euler_ahrs.thz * 5730);

			if (gTHR < MIN_THR) {
				euler_rc.thz = 0;
				euler_ahrs.thz = 0;
			}

			euler_rc_fil.thx = euler_rc.thx;
			euler_rc_fil.thy = euler_rc.thy;
			euler_rc_fil.thz = euler_rc.thz;

			FlightControlPID_OuterLoop(&euler_rc_fil, &euler_ahrs, &ahrs, &pid);

			/* Added for debug on UART*/
			/* Remocon ELE, AIL, RUD, THR, AHRS Euler angle x and y axis, Remocon Euler angle x and y axis */
			//PRINTF("%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\n", gELE, gAIL, gRUD, gTHR, euler_ahrs.thx * 57.3f, euler_ahrs.thy * 57.3f, euler_rc.thx * 57.3f, euler_rc.thy * 57.3f);
			/* Remocon ELE, AIL, RUD, THR, Motor1_pwm, AHRS Euler angle x and y axis */
			//PRINTF("%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\n", gELE, gAIL, gRUD, gTHR, motor_pwm.motor1_pwm, euler_ahrs.thx * 57.3f, euler_ahrs.thy * 57.3f, euler_rc.thx * 57.3f, euler_rc.thy * 57.3f);
			//PRINTF("%d\t%d\t%d\t%d\n", gELE, gAIL, gRUD, gTHR);
			/* Remocon THR, Acc and Gyro FIFO data x and y axis, AHRS Euler angle x and y axis, Remocon Euler angle x and y axis*/
			//PRINTF("%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", gTHR, acc_ahrs.AXIS_X, acc_ahrs.AXIS_Y, gyro_ahrs.AXIS_X, gyro_ahrs.AXIS_Y, euler_ahrs.thx * 57.3f, euler_ahrs.thy * 57.3f, euler_rc.thx * 57.3f, euler_rc.thy * 57.3f);
			/* MEMS Accelerometer RAW data */
			//PRINTF("%d\t%d\t%d\t\n", acc.AXIS_X, acc.AXIS_Y, acc.AXIS_Z);
			/* Pressure data on UART for debug*/
			//PRINTF("Pressure [atm] = %f\n\n",pre);
			/* Magnetometer data on UART for debug*/
			//PRINTF("Magnetometer X = %d\tY = %d\tZ = %d\n\n", mag.AXIS_X, mag.AXIS_Y, mag.AXIS_Z);

		}

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
			ch_flag = 1;
		}

		if (isTimerEventExist(&tim))    // Check if a timer event is present
				{

			ClearTimer(&tim);           // Clear current event;

			count2++;

			mytimcnt++;
			if (rc_connection_flag && rc_enable_motor) {
				if (mytimcnt % 50 == 0)
					BSP_LED_On(LED2);
			} else {
				if (mytimcnt % 50 == 0)
					BSP_LED_Toggle(LED2);
			}
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 32767;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
#ifdef MOTOR_DC
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
#endif
#ifdef MOTOR_ESC
  	htim4.Instance = TIM4;
    htim4.Init.Prescaler = 100;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 2075;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
      Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
      Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }
#endif
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 51;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LPS22HH_CS_GPIO_Port, LPS22HH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLE_RSTN_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LSM6DSR_CS_GPIO_Port, LSM6DSR_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LPS22HH_CS_Pin */
  GPIO_InitStruct.Pin = LPS22HH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LPS22HH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_Monitor_Pin */
  GPIO_InitStruct.Pin = USB_Monitor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_Monitor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_IRQ_Pin */
  GPIO_InitStruct.Pin = BLE_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_RSTN_Pin */
  GPIO_InitStruct.Pin = BLE_RSTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_RSTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSR_CS_Pin */
  GPIO_InitStruct.Pin = LSM6DSR_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LSM6DSR_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * This function read sensor data and prepare data for proper coordinate system
 * according to definition of COORDINATE_SYSTEM
 * The unit of each data are:
 *      Acc - mg
 *      Gyro - mdps
 *      Mag - mguass
 */
void ReadSensorRawData( AxesRaw_TypeDef *acc, AxesRaw_TypeDef *gyro,  float *pre)
{
    int32_t t1;
    CUSTOM_MOTION_SENSOR_AxesRaw_t acc_temp_int16, gyro_temp_int16;            /* Data Type int16_t */
    AxesRaw_TypeDef acc_temp, gyro_temp;
    /* Data Type int32_t */
    // Read data is in mg unit
    CUSTOM_MOTION_SENSOR_GetAxesRaw(CUSTOM_LSM6DSR_0, MOTION_ACCELERO, &acc_temp_int16);
    acc_temp.AXIS_X = (int32_t) acc_temp_int16.x;                /* Casting data to int32_t */
    acc_temp.AXIS_Y = (int32_t) acc_temp_int16.y;
    acc_temp.AXIS_Z = (int32_t) acc_temp_int16.z;
    // Read data is in mdps unit
    CUSTOM_MOTION_SENSOR_GetAxesRaw(CUSTOM_LSM6DSR_0, MOTION_GYRO, &gyro_temp_int16);
    gyro_temp.AXIS_X = (int32_t) gyro_temp_int16.x;                /* Casting data to int32_t */
    gyro_temp.AXIS_Y = (int32_t) gyro_temp_int16.y;
    gyro_temp.AXIS_Z = (int32_t) gyro_temp_int16.z;

    if (USE_PRESSURE_SENSOR)
    	CUSTOM_ENV_SENSOR_GetValue(CUSTOM_LPS22HH_0,ENV_PRESSURE, pre);
    else
        pre = 0;


    if (COORDINATE_SYSTEM == 1)
    {
        // convert acc
        t1 = acc->AXIS_X;
        acc->AXIS_X = acc->AXIS_Y;
        acc->AXIS_Y = -t1;
        // convert gyro
        t1 = gyro->AXIS_X;
        gyro->AXIS_X = gyro->AXIS_Y;
        gyro->AXIS_Y = -t1;

    }
    else if (COORDINATE_SYSTEM == 2)
    {
        // No need to convert in this case
    }
    else if (COORDINATE_SYSTEM == 3)
    {

      acc->AXIS_X = -acc_temp.AXIS_Y;
      acc->AXIS_Y = acc_temp.AXIS_X;
      acc->AXIS_Z = acc_temp.AXIS_Z;

      gyro->AXIS_X = -gyro_temp.AXIS_Y;
      gyro->AXIS_Y = gyro_temp.AXIS_X;
      gyro->AXIS_Z = gyro_temp.AXIS_Z;


    }
    else if (COORDINATE_SYSTEM == 4)
    {
        // convert acc
        acc->AXIS_X = - acc->AXIS_X;
        acc->AXIS_Y = - acc->AXIS_Y;
        // convert gyro
        gyro->AXIS_X = - gyro->AXIS_X;
        gyro->AXIS_Y = - gyro->AXIS_Y;

    }
}
/*
 *  Handle Timer9 interrupt @ 800Hz
 *  Set the event flag and increase time index
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(sensor_init_cali == 0)
  {
    sensor_init_cali_count++;

    if(sensor_init_cali_count > 800)
    {
      // Read sensor data and prepare for specific coodinate system
      ReadSensorRawData( &acc, &gyro, &press);

      acc_off_calc.AXIS_X += acc.AXIS_X;
      acc_off_calc.AXIS_Y += acc.AXIS_Y;
      acc_off_calc.AXIS_Z += acc.AXIS_Z;

      gyro_off_calc.AXIS_X += gyro.AXIS_X;
      gyro_off_calc.AXIS_Y += gyro.AXIS_Y;
      gyro_off_calc.AXIS_Z += gyro.AXIS_Z;

      if (sensor_init_cali_count >= 1600)
      {
        acc_offset.AXIS_X = (int32_t) (acc_off_calc.AXIS_X * 0.00125f);
        acc_offset.AXIS_Y = (int32_t) (acc_off_calc.AXIS_Y * 0.00125f);
        acc_offset.AXIS_Z = (int32_t) (acc_off_calc.AXIS_Z * 0.00125f);

        gyro_offset.AXIS_X = (int32_t) (gyro_off_calc.AXIS_X * 0.00125f);
        gyro_offset.AXIS_Y = (int32_t) (gyro_off_calc.AXIS_Y * 0.00125f);
        gyro_offset.AXIS_Z = (int32_t) (gyro_off_calc.AXIS_Z * 0.00125f);

        acc_off_calc.AXIS_X = 0;
        acc_off_calc.AXIS_Y = 0;
        acc_off_calc.AXIS_Z = 0;
        gyro_off_calc.AXIS_X = 0;
        gyro_off_calc.AXIS_Y = 0;
        gyro_off_calc.AXIS_Z = 0;

        sensor_init_cali_count = 0;
        sensor_init_cali = 1;
      }
    }
  }

  if(sensor_init_cali == 1)
  {
    tim9_cnt++;
    tim9_cnt2++;

    // Read sensor data and prepare for specific coodinate system
    ReadSensorRawData( &acc, &gyro, &press);

    if (rc_cal_flag == 1)
    {
      acc_off_calc.AXIS_X += acc.AXIS_X;
      acc_off_calc.AXIS_Y += acc.AXIS_Y;
      acc_off_calc.AXIS_Z += acc.AXIS_Z;

      gyro_off_calc.AXIS_X += gyro.AXIS_X;
      gyro_off_calc.AXIS_Y += gyro.AXIS_Y;
      gyro_off_calc.AXIS_Z += gyro.AXIS_Z;

      rc_cal_cnt++;

      if (rc_cal_cnt >= 800)
      {
        acc_offset.AXIS_X = (int32_t) (acc_off_calc.AXIS_X * 0.00125f);
        acc_offset.AXIS_Y = (int32_t) (acc_off_calc.AXIS_Y * 0.00125f);
        acc_offset.AXIS_Z = (int32_t) (acc_off_calc.AXIS_Z * 0.00125f);

        gyro_offset.AXIS_X = (int32_t) (gyro_off_calc.AXIS_X * 0.00125f);
        gyro_offset.AXIS_Y = (int32_t) (gyro_off_calc.AXIS_Y * 0.00125f);
        gyro_offset.AXIS_Z = (int32_t) (gyro_off_calc.AXIS_Z * 0.00125f);

        acc_off_calc.AXIS_X = 0;
        acc_off_calc.AXIS_Y = 0;
        acc_off_calc.AXIS_Z = 0;
        gyro_off_calc.AXIS_X = 0;
        gyro_off_calc.AXIS_Y = 0;
        gyro_off_calc.AXIS_Z = 0;

        rc_cal_cnt = 0;
        rc_cal_flag = 0;
      }
    }

    acc.AXIS_X -= acc_offset.AXIS_X;
    acc.AXIS_Y -= acc_offset.AXIS_Y;
    acc.AXIS_Z -= (acc_offset.AXIS_Z - 1000);
    gyro.AXIS_X -= gyro_offset.AXIS_X;
    gyro.AXIS_Y -= gyro_offset.AXIS_Y;
    gyro.AXIS_Z -= gyro_offset.AXIS_Z;

    // Save filtered data to acc_FIFO
    acc_FIFO[tim9_cnt2-1].AXIS_X = acc.AXIS_X;
    acc_FIFO[tim9_cnt2-1].AXIS_Y = acc.AXIS_Y;
    acc_FIFO[tim9_cnt2-1].AXIS_Z = acc.AXIS_Z;

    // IIR Filtering on gyro
    gyro_fil.AXIS_X = gyro_fil_coeff.b0*gyro.AXIS_X + gyro_fil_coeff.b1*gyro_x_pre[0].AXIS_X + gyro_fil_coeff.b2*gyro_x_pre[1].AXIS_X
                                                    + gyro_fil_coeff.a1*gyro_y_pre[0].AXIS_X + gyro_fil_coeff.a2*gyro_y_pre[1].AXIS_X;
    gyro_fil.AXIS_Y = gyro_fil_coeff.b0*gyro.AXIS_Y + gyro_fil_coeff.b1*gyro_x_pre[0].AXIS_Y + gyro_fil_coeff.b2*gyro_x_pre[1].AXIS_Y
                                                    + gyro_fil_coeff.a1*gyro_y_pre[0].AXIS_Y + gyro_fil_coeff.a2*gyro_y_pre[1].AXIS_Y;
    gyro_fil.AXIS_Z = gyro_fil_coeff.b0*gyro.AXIS_Z + gyro_fil_coeff.b1*gyro_x_pre[0].AXIS_Z + gyro_fil_coeff.b2*gyro_x_pre[1].AXIS_Z
                                                    + gyro_fil_coeff.a1*gyro_y_pre[0].AXIS_Z + gyro_fil_coeff.a2*gyro_y_pre[1].AXIS_Z;
    // Shift IIR filter state
    for(int i=1;i>0;i--)
    {
      gyro_x_pre[i].AXIS_X = gyro_x_pre[i-1].AXIS_X;
      gyro_x_pre[i].AXIS_Y = gyro_x_pre[i-1].AXIS_Y;
      gyro_x_pre[i].AXIS_Z = gyro_x_pre[i-1].AXIS_Z;
      gyro_y_pre[i].AXIS_X = gyro_y_pre[i-1].AXIS_X;
      gyro_y_pre[i].AXIS_Y = gyro_y_pre[i-1].AXIS_Y;
      gyro_y_pre[i].AXIS_Z = gyro_y_pre[i-1].AXIS_Z;
    }
    gyro_x_pre[0].AXIS_X = gyro.AXIS_X;
    gyro_x_pre[0].AXIS_Y = gyro.AXIS_Y;
    gyro_x_pre[0].AXIS_Z = gyro.AXIS_Z;
    gyro_y_pre[0].AXIS_X = gyro_fil.AXIS_X;
    gyro_y_pre[0].AXIS_Y = gyro_fil.AXIS_Y;
    gyro_y_pre[0].AXIS_Z = gyro_fil.AXIS_Z;

    //  Save filtered data to gyro_FIFO
    gyro_FIFO[tim9_cnt2-1].AXIS_X = gyro_fil.AXIS_X;
    gyro_FIFO[tim9_cnt2-1].AXIS_Y = gyro_fil.AXIS_Y;
    gyro_FIFO[tim9_cnt2-1].AXIS_Z = gyro_fil.AXIS_Z;


    if(tim9_cnt2 == FIFO_Order)
    {
      tim9_cnt2 = 0;
      tim9_event_flag = 1;
      for(int i=0;i<FIFO_Order;i++)
      {
        acc_ahrs_FIFO[i].AXIS_X = acc_FIFO[i].AXIS_X;
        acc_ahrs_FIFO[i].AXIS_Y = acc_FIFO[i].AXIS_Y;
        acc_ahrs_FIFO[i].AXIS_Z = acc_FIFO[i].AXIS_Z;
        gyro_ahrs_FIFO[i].AXIS_X = gyro_FIFO[i].AXIS_X;
        gyro_ahrs_FIFO[i].AXIS_Y = gyro_FIFO[i].AXIS_Y;
        gyro_ahrs_FIFO[i].AXIS_Z = gyro_FIFO[i].AXIS_Z;
      }
    }


      gyro_rad.gx = ((float)gyro_fil.AXIS_X)*((float)COE_MDPS_TO_RADPS);
      gyro_rad.gy = ((float)gyro_fil.AXIS_Y)*((float)COE_MDPS_TO_RADPS);
      gyro_rad.gz = ((float)gyro_fil.AXIS_Z)*((float)COE_MDPS_TO_RADPS);

      euler_ahrs.thz += gyro_rad.gz*PID_SAMPLING_TIME;

      if(gTHR<MIN_THR)
      {
        euler_rc.thz = 0;
        euler_ahrs.thz = 0;
      }

      if (rc_connection_flag && rc_enable_motor)
      {   // Do PID Control
        FlightControlPID_innerLoop(&euler_rc_fil, &gyro_rad, &ahrs, &pid, &motor_pwm);
      }
      else
      {
        // set motor output zero
        set_motor_pwm_zero(&motor_pwm);
      }

      if(gTHR<MIN_THR)
      {
        set_motor_pwm_zero(&motor_pwm);
      }

      set_motor_pwm(&motor_pwm);      /* To comment if want to debug remocon calibration switching off the motors */
  }
}


/**
* @brief  Configures LEDs.
* @param  Led: LED to be configured.
*          This parameter can be one of the following values:
*            @arg  LED1
* @retval None
*/
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;


  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}


/**
* @brief  DeInit LEDs.
* @param  Led: LED to be configured.
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @note Led DeInit does not disable the GPIO clock nor disable the Mfx
* @retval None
*/
void BSP_LED_DeInit(Led_TypeDef Led)
{

}

/**
* @brief  Turns selected LED On.
* @param  Led: LED to be set on
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_On(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
* @brief  Turns selected LED Off.
* @param  Led: LED to be set off
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Off(Led_TypeDef Led)
{
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}


/**
* @brief  Toggles the selected LED.
* @param  Led: LED to be toggled
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);

}


/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
/*void HAL_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
    case BLE_IRQ_EXTI_IRQn:
      HCI_Isr();
      HCI_ProcessEvent=1;
    break;
  }
}*/



void BlueNRG_Init(void)
{

  int ret = 1;
  uint8_t  hwVersion=0;
  uint16_t fwVersion=0;

  PRINTF("****** START BLE TESTS ******\r\n");
  BNRG_SPI_Init();

  /* Commented on Jan 15, 2020 */
 // uint8_t tmp_bdaddr[6]= {MAC_BLUEMS};
 // int32_t i;
  /* Commented on Jan 15, 2020 */
  //for(i=0;i<6;i++)
  //  bdaddr[i] = tmp_bdaddr[i];

  /* Added on Jan 15, 2020 */
  bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
  bdaddr[1] = (STM32_UUID[0]    )&0xFF;
  bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
  bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
  bdaddr[4] = (hwVersion > 0x30) ?
            ((((0x34-48)*10) + (0x30-48)+100)&0xFF) :
            ((((0x34-48)*10) + (0x30-48)    )&0xFF) ;
  bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */

  /* Initialize the BlueNRG HCI */
  HCI_Init();

 /* Reset BlueNRG hardware */
  BlueNRG_RST();

  /* get the BlueNRG HW and FW versions */
  PRINTF("\r\nReading BlueNRG version ...\r\n");
  if (getBlueNRGVersion(&hwVersion, &fwVersion)== BLE_STATUS_SUCCESS)
  {

    /*
     * Reset BlueNRG again otherwise it will fail.
     */
    BlueNRG_RST();

    //PRINTF("Setting Pubblic Address...\r\n");
    /* Commented on Jan 15, 2020 */
    //ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
    //                                CONFIG_DATA_PUBADDR_LEN,
    //                                bdaddr);
    //if(ret){
    //  testStatus = COMPONENT_ERROR;
    //  PRINTF("\r\nSetting Pubblic BD_ADDR failed *****\r\n");
    //  goto fail;
    //}

    PRINTF("GATT Initializzation...\r\n");
    ret = aci_gatt_init();
    if(ret){
      testStatus = COMPONENT_ERROR;
      PRINTF("\r\nGATT_Init failed ****\r\n");
      goto fail;
    }

//    ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
//                                     7/*strlen(BoardName)*/, (uint8_t *)BoardName);
//
//    if(ret){
//       PRINTF("\r\naci_gatt_update_char_value failed\r\n");
//      while(1);
//    }

    /* Set the GAP INIT like X-NUCLEO-IDB05A1 eval board  since using same SPBTLE_RF module*/
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

    if(ret != BLE_STATUS_SUCCESS){
      PRINTF("\r\nGAP_Init failed\r\n");
      goto fail;
    }

    // Added Jan 10th
    ret = hci_le_set_random_address(bdaddr);
    // Added Jan 10th
    const char BoardName[7] = {NAME_BLUEMS};
    // Added Jan 10th
    ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                       7/*strlen(BoardName)*/, (uint8_t *)BoardName);

    PRINTF("GAP setting Authentication ....\r\n");
    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                       OOB_AUTH_DATA_ABSENT,
                                       NULL, 7, 16,
                                       USE_FIXED_PIN_FOR_PAIRING, 123456,
                                       BONDING);
    if (ret != BLE_STATUS_SUCCESS) {
      testStatus = COMPONENT_ERROR;
       PRINTF("\r\nGAP setting Authentication failed ******\r\n");
       goto fail;
    }

    PRINTF("SERVER: BLE Stack Initialized \r\n"
           "Board HWver=%d, FWver=%d.%d.%c\r\n"
           "BoardMAC = %x:%x:%x:%x:%x:%x\r\n",
           hwVersion,
           fwVersion>>8,
           (fwVersion>>4)&0xF,
           (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
           bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

    /* Set output power level */
    aci_hal_set_tx_power_level(1,4);    /* -2.1dBm */

    ret = Add_ConsoleW2ST_Service();
    if(ret == BLE_STATUS_SUCCESS)
       PRINTF("Console Service W2ST added successfully\r\n");
    else{
       testStatus = COMPONENT_ERROR;
       PRINTF("\r\nError while adding Console Service W2ST\r\n");
    }

    ret = Add_ConfigW2ST_Service();
    if(ret == BLE_STATUS_SUCCESS)
       PRINTF("Config  Service W2ST added successfully\r\n");
    else{
       testStatus = COMPONENT_ERROR;
       PRINTF("\r\nError while adding Config Service W2ST\r\n");
    }

    PRINTF("\r\nAll test passed!\r\n");
  }
  else {
       testStatus = COMPONENT_ERROR;
       PRINTF("\r\nError in BlueNRG tests. ******\r\n");
  }
  PRINTF("****** END BLE TESTS ******\r\n");
  return;

fail:
  testStatus = COMPONENT_ERROR;
  return;
}



/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_HWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     PRINTF("HW      Service W2ST added successfully\r\n");
  } else {
     PRINTF("\r\nError while adding HW Service W2ST\r\n");
  }

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     PRINTF("Console Service W2ST added successfully\r\n");
  } else {
     PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) {
     PRINTF("Config  Service W2ST added successfully\r\n");
  } else {
     PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  AxesRaw_TypeDef ACC_Value;
  AxesRaw_TypeDef GYR_Value;



  ACC_Value.AXIS_X = acc.AXIS_X;
  ACC_Value.AXIS_Y = acc.AXIS_Y;
  ACC_Value.AXIS_Z = acc.AXIS_Z;
  GYR_Value.AXIS_X = gyro.AXIS_X;
  GYR_Value.AXIS_Y = gyro.AXIS_Y;
  GYR_Value.AXIS_Z = gyro.AXIS_Z;



  /*Debug */
  //PRINTF("ACC[X, Y, Z]: %d\t%d\t%d\t\r\n", ACC_Value.AXIS_X, ACC_Value.AXIS_Y, ACC_Value.AXIS_Z);
  //PRINTF("GYRO[X, Y, Z]: %d\t%d\t%d\t\r", GYR_Value.AXIS_X, GYR_Value.AXIS_Y, GYR_Value.AXIS_Z);
  //PRINTF("MAG[X, Y, Z]: %d\t%d\t%d\t\n", MAG_Value.AXIS_X, MAG_Value.AXIS_Y, MAG_Value.AXIS_Z);

  AccGyroMag_Update(&ACC_Value, &GYR_Value, &GYR_Value);

}

static void SendBattEnvData(void)
{
   int32_t decPart, intPart;
   int32_t PressToSend=0;
   uint16_t BattToSend=0;
   int16_t RSSIToSend=0, TempToSend=0;
   int8_t rssi;
   uint16_t conn_handle;

   HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
        {
            VBAT_Sense = HAL_ADC_GetValue(&hadc1);
            VBAT = (((VBAT_Sense*3.3)/4095)*(BAT_RUP+BAT_RDW))/BAT_RDW;
            //PRINTF("Battery voltage = %fV\n\n", VBAT);
        }
    HAL_ADC_Stop(&hadc1);

    // Pressure to Send
    MCR_BLUEMS_F2I_2D(press, intPart, decPart);
    PressToSend=intPart*100+decPart;

    // Battery to Send
    VBAT = ((VBAT - 3.7f)*100.0f)/(4.2f-3.7f);
    if (VBAT < 0) VBAT = 0;
    MCR_BLUEMS_F2I_1D((int32_t)VBAT, intPart, decPart);
    BattToSend = intPart*10+decPart;
    if (BattToSend > 1000){
      BattToSend =1000;
    }

    // Temperature to Send
    MCR_BLUEMS_F2I_1D(temperature, intPart, decPart);
    TempToSend = intPart*10+decPart;

    // RSSI to send
    hci_read_rssi(&conn_handle, &rssi);
    RSSIToSend = (int16_t)rssi*10;

    Batt_Env_RSSI_Update(PressToSend,BattToSend,(int16_t) TempToSend,RSSIToSend );

}


static void SendArmingData(void)
{
   ARMING_Update(rc_enable_motor);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
