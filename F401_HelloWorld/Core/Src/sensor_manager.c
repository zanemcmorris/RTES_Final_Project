#include "sensor_manager.h"

#include "cmsis_os2.h"
#include "lsm6dsr.h"
#include "lps22hh.h"
#include "vl53l0x_platform.h"

#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>

#include "vl53l0x_api.h"

extern osMutexId_t IMUDataMutexID;
extern osMutexId_t altitudeDataMutexID;

static volatile debug_output_mode_t g_debug_output_mode = DEBUG_OUT_STAT;

static LSM6DSR_Object_t MotionSensor;
static LPS22HH_Object_t BaroSensor;
static VL53L0X_Dev_t g_vl53l0x_dev;

static volatile uint8_t g_spi2_dma_done = 0;
static volatile uint8_t g_spi2_dma_error = 0;

static spi_device_t imu_dev = { .hspi = &hspi2, .cs_port = IMU_CS_PORT,
		.cs_pin = IMU_CS_PIN };
static spi_device_t baro_dev = { .hspi = &hspi2, .cs_port = BARO_CS_PORT,
		.cs_pin = BARO_CS_PIN };

raw_imu_sample_t g_raw_imu = { 0 };
processed_imu_sample_t g_processed_imu = { 0 };
raw_baro_sample_t g_raw_baro = { 0 };
processed_baro_sample_t g_processed_baro = { 0 };

static float g_gyro_bias_x_mdps = 0.0f;
static float g_gyro_bias_y_mdps = 0.0f;
static float g_gyro_bias_z_mdps = 0.0f;

static uint32_t g_imu_task_loops = 0;
static uint32_t g_acc_ready_count = 0;
static uint32_t g_gyro_ready_count = 0;
static uint32_t g_acc_read_count = 0;
static uint32_t g_gyro_read_count = 0;

static uint32_t g_baro_status_checks = 0;
static uint32_t g_baro_press_overrun_count = 0;
static uint32_t g_baro_temp_overrun_count = 0;

static uint32_t g_imu_fifo_status_checks = 0;
static uint32_t g_imu_fifo_overrun_count = 0;
static uint32_t g_imu_fifo_max_samples = 0;
static uint16_t g_imu_fifo_last_samples = 0;

static uint32_t g_imu_fifo_drained_samples = 0;

static raw_tof_sample_t g_raw_tof = { 0 };
static processed_tof_sample_t g_processed_tof = { 0 };

static uint32_t micros(void) {
	return (uint32_t) ((HAL_GetTick() * 1000U)
			+ ((SysTick->LOAD - SysTick->VAL) * 1000U) / (SysTick->LOAD + 1U));
}

static void spi2_deselect_all(void) {
	HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BARO_CS_PORT, BARO_CS_PIN, GPIO_PIN_SET);
}

static void spi_dev_select(spi_device_t *dev) {
	spi2_deselect_all();
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void spi_dev_deselect(spi_device_t *dev) {
	HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
		g_spi2_dma_done = 1;
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
		g_spi2_dma_error = 1;
	}
}

static int32_t BSP_SPI2_Init(void) {
	spi2_deselect_all();
	return 0;
}

static int32_t BSP_SPI2_DeInit(void) {
	spi2_deselect_all();
	return 0;
}

static int32_t spi_bus_read_reg(spi_device_t *dev, uint8_t reg, uint8_t *pData,
		uint16_t Length) {
	uint8_t txbuf[32];
	uint8_t rxbuf[32];
	uint32_t timeout_start;

	if (Length == 0 || Length > (sizeof(txbuf) - 1))
		return LSM6DSR_ERROR;

	txbuf[0] = reg | 0x80;
	memset(&txbuf[1], 0x00, Length);
	memset(rxbuf, 0x00, Length + 1);

	g_spi2_dma_done = 0;
	g_spi2_dma_error = 0;

	spi_dev_select(dev);

	if (HAL_SPI_TransmitReceive_DMA(dev->hspi, txbuf, rxbuf, Length + 1)
			!= HAL_OK) {
		spi_dev_deselect(dev);
		return LSM6DSR_ERROR;
	}

	timeout_start = HAL_GetTick();

	while (g_spi2_dma_done == 0 && g_spi2_dma_error == 0) {
		if ((HAL_GetTick() - timeout_start) > 100) {
			HAL_SPI_Abort(dev->hspi);
			spi_dev_deselect(dev);
			return LSM6DSR_ERROR;
		}
	}

	spi_dev_deselect(dev);

	if (g_spi2_dma_error != 0) {
		return LSM6DSR_ERROR;
	}

	memcpy(pData, &rxbuf[1], Length);
	return LSM6DSR_OK;
}

static int32_t spi_bus_write_reg(spi_device_t *dev, uint8_t reg, uint8_t *pData,
		uint16_t Length) {
	uint8_t txbuf[32];

	if (Length == 0 || Length > (sizeof(txbuf) - 1))
		return LSM6DSR_ERROR;

	txbuf[0] = reg;
	memcpy(&txbuf[1], pData, Length);

	spi_dev_select(dev);

	if (HAL_SPI_Transmit(dev->hspi, txbuf, Length + 1, 100) != HAL_OK) {
		spi_dev_deselect(dev);
		return LSM6DSR_ERROR;
	}

	spi_dev_deselect(dev);
	return LSM6DSR_OK;
}

static int32_t IMU_SPI2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length) {
	(void) Addr;
	return spi_bus_read_reg(&imu_dev, (uint8_t) Reg, pData, Length);
}

static int32_t IMU_SPI2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length) {
	(void) Addr;
	return spi_bus_write_reg(&imu_dev, (uint8_t) Reg, pData, Length);
}

static int32_t BARO_SPI2_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length) {
	(void) Addr;
	return spi_bus_read_reg(&baro_dev, (uint8_t) Reg, pData, Length);
}

static int32_t BARO_SPI2_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData,
		uint16_t Length) {
	(void) Addr;
	return spi_bus_write_reg(&baro_dev, (uint8_t) Reg, pData, Length);
}

static HAL_StatusTypeDef vl53l0x_read_u8(uint8_t reg, uint8_t *value) {
	return HAL_I2C_Mem_Read(&hi2c2,
	VL53L0X_I2C_ADDR_8BIT, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 100);
}

static void vl53l0x_setup_device_struct(void) {
	memset(&g_vl53l0x_dev, 0, sizeof(g_vl53l0x_dev));
	g_vl53l0x_dev.I2cDevAddr = VL53L0X_I2C_ADDR_8BIT;
	g_vl53l0x_dev.comms_type = 1;
	g_vl53l0x_dev.comms_speed_khz = 400;
}

int32_t MX_LSM6DSR_Init(void) {
	LSM6DSR_IO_t io_ctx;
	uint8_t id = 0;
	int32_t fullScale = 2;

	io_ctx.Init = BSP_SPI2_Init;
	io_ctx.DeInit = BSP_SPI2_DeInit;
	io_ctx.BusType = LSM6DSR_SPI_4WIRES_BUS;
	io_ctx.Address = 0;
	io_ctx.WriteReg = IMU_SPI2_WriteReg;
	io_ctx.ReadReg = IMU_SPI2_ReadReg;

	if (LSM6DSR_RegisterBusIO(&MotionSensor, &io_ctx) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_Init(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_ReadID(&MotionSensor, &id) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (id != 0x6B)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_GYRO_Enable(&MotionSensor) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_ACC_SetOutputDataRate(&MotionSensor,
			LSM6DSR_XL_ODR_104Hz) != LSM6DSR_OK)
		return LSM6DSR_ERROR;
	if (LSM6DSR_ACC_SetFullScale(&MotionSensor, fullScale) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (LSM6DSR_GYRO_SetOutputDataRate(&MotionSensor,
			LSM6DSR_XL_ODR_104Hz) != LSM6DSR_OK)
		return LSM6DSR_ERROR; // TODO: Make sure these enum defns match the #defined period and freq
	if (LSM6DSR_GYRO_SetFullScale(&MotionSensor, LSM6DSR_250dps) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	return LSM6DSR_OK;
}

static int32_t MX_LSM6DSR_FIFO_Test_Init(void) {
	if (lsm6dsr_fifo_mode_set(&(MotionSensor.Ctx),
			LSM6DSR_BYPASS_MODE) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (lsm6dsr_fifo_xl_batch_set(&(MotionSensor.Ctx),
			LSM6DSR_XL_BATCHED_AT_104Hz) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (lsm6dsr_fifo_gy_batch_set(&(MotionSensor.Ctx),
			LSM6DSR_GY_BATCHED_AT_104Hz) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	if (lsm6dsr_fifo_mode_set(&(MotionSensor.Ctx),
			LSM6DSR_STREAM_MODE) != LSM6DSR_OK)
		return LSM6DSR_ERROR;

	return LSM6DSR_OK;
}

int32_t MX_LPS22HH_Init(void) {
	LPS22HH_IO_t io_ctx;
	uint8_t id = 0;

	io_ctx.Init = BSP_SPI2_Init;
	io_ctx.DeInit = BSP_SPI2_DeInit;
	io_ctx.BusType = LPS22HH_SPI_4WIRES_BUS;
	io_ctx.Address = 0;
	io_ctx.WriteReg = BARO_SPI2_WriteReg;
	io_ctx.ReadReg = BARO_SPI2_ReadReg;
	io_ctx.Delay = HAL_Delay;

	if (LPS22HH_RegisterBusIO(&BaroSensor, &io_ctx) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_Init(&BaroSensor) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_ReadID(&BaroSensor, &id) != LPS22HH_OK)
		return LPS22HH_ERROR;

	if (id != 0xB3)
		return LPS22HH_ERROR;

	if (LPS22HH_PRESS_Enable(&BaroSensor) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_TEMP_Enable(&BaroSensor) != LPS22HH_OK)
		return LPS22HH_ERROR;

	if (lps22hh_lp_bandwidth_set(&(BaroSensor.Ctx),
			LPS22HH_LPF_ODR_DIV_9) != LPS22HH_OK)
		return LPS22HH_ERROR;

	if (LPS22HH_PRESS_SetOutputDataRate(&BaroSensor, 10.0f) != LPS22HH_OK)
		return LPS22HH_ERROR;
	if (LPS22HH_TEMP_SetOutputDataRate(&BaroSensor, 10.0f) != LPS22HH_OK)
		return LPS22HH_ERROR;

	return LPS22HH_OK;
}

static VL53L0X_Error vl53l0x_api_init_device(void) {
	VL53L0X_Error status;
	uint8_t vhv_settings = 0;
	uint8_t phase_cal = 0;
	uint32_t ref_spad_count = 0;
	uint8_t is_aperture_spads = 0;

	vl53l0x_setup_device_struct();

	status = VL53L0X_DataInit(&g_vl53l0x_dev);
	if (status != VL53L0X_ERROR_NONE) {
		return status;
	}

	status = VL53L0X_StaticInit(&g_vl53l0x_dev);
	if (status != VL53L0X_ERROR_NONE) {
		return status;
	}

	status = VL53L0X_PerformRefSpadManagement(&g_vl53l0x_dev, &ref_spad_count,
			&is_aperture_spads);
	if (status != VL53L0X_ERROR_NONE) {
		return status;
	}

	status = VL53L0X_PerformRefCalibration(&g_vl53l0x_dev, &vhv_settings,
			&phase_cal);
	if (status != VL53L0X_ERROR_NONE) {
		return status;
	}

	status = VL53L0X_SetDeviceMode(&g_vl53l0x_dev,
	VL53L0X_DEVICEMODE_SINGLE_RANGING);
	return status;
}

static void preprocess_imu_sample(const raw_imu_sample_t *raw,
		processed_imu_sample_t *proc) {
	proc->timestamp_us = raw->timestamp_us;

	// milli-g to g
	proc->accel_x_g = raw->accel_x_mg / 1000.0f;
	proc->accel_y_g = raw->accel_y_mg / 1000.0f;
	proc->accel_z_g = raw->accel_z_mg / 1000.0f;

	// g to meter per second^2
	proc->accel_x_mps2 = proc->accel_x_g * GRAVITY_MPS2;
	proc->accel_y_mps2 = proc->accel_y_g * GRAVITY_MPS2;
	proc->accel_z_mps2 = proc->accel_z_g * GRAVITY_MPS2;

	// Riemann sum acceleration (mps2 * t = mps)
	proc->vel_x_mps = proc->accel_x_mps2 * (IMU_SAMPLING_PERIOD_MS / 1000.0);
	proc->vel_y_mps = proc->accel_y_mps2 * (IMU_SAMPLING_PERIOD_MS / 1000.0);
	proc->vel_z_mps = proc->accel_z_mps2 * (IMU_SAMPLING_PERIOD_MS / 1000.0);

	// Riemann sum velocity (mps * t = m)
	proc->pos_x_m = proc->vel_x_mps * (IMU_SAMPLING_PERIOD_MS / 1000.0);
	proc->pos_y_m = proc->vel_y_mps * (IMU_SAMPLING_PERIOD_MS / 1000.0);
	proc->pos_z_m = proc->vel_z_mps * (IMU_SAMPLING_PERIOD_MS / 1000.0);

	// milli-dps to dps with gyro calibration remove
	proc->gyro_x_dps = (raw->gyro_x_mdps - g_gyro_bias_x_mdps) / 1000.0f;
	proc->gyro_y_dps = (raw->gyro_y_mdps - g_gyro_bias_y_mdps) / 1000.0f;
	proc->gyro_z_dps = (raw->gyro_z_mdps - g_gyro_bias_z_mdps) / 1000.0f;

	// dps to rps
	proc->gyro_x_rps = proc->gyro_x_dps * DEG_TO_RAD;
	proc->gyro_y_rps = proc->gyro_y_dps * DEG_TO_RAD;
	proc->gyro_z_rps = proc->gyro_z_dps * DEG_TO_RAD;

	// Riemann sum rps (rad per second * t = radian)
	// Also with combined_roll and pitch feedback
	proc->gyro_x_rad_abs = proc->combined_roll + proc->gyro_x_rps
			* (IMU_SAMPLING_PERIOD_MS / 1000.0);
	proc->gyro_y_rad_abs = proc->combined_pitch + proc->gyro_y_rps
			* (IMU_SAMPLING_PERIOD_MS / 1000.0);
	proc->gyro_z_rad_abs += proc->gyro_z_rps
			* (IMU_SAMPLING_PERIOD_MS / 1000.0);

	proc->accel_roll = atan2f(proc->accel_y_g, proc->accel_z_g);
	proc->accel_pitch = atan2f(-1 * proc->accel_x_g,
			sqrtf(proc->accel_y_g * proc->accel_y_g
							+ proc->accel_z_g * proc->accel_z_g));

	// Complementary filter
	proc->combined_roll = 0.97 * proc->gyro_x_rad_abs + 0.03 * proc->accel_roll;
	proc->combined_pitch = 0.97 * proc->gyro_y_rad_abs + 0.03 * proc->accel_pitch;

}

static float pressure_to_altitude_m(float pressure_hpa) {
	if (pressure_hpa <= 0.0f) {
		return 0.0f;
	}

	return 44330.0f * (1.0f - powf(pressure_hpa / SEA_LEVEL_HPA, 0.1903f));
}

static void preprocess_baro_sample(const raw_baro_sample_t *raw,
		processed_baro_sample_t *proc) {
	proc->timestamp_us = raw->timestamp_us;
	proc->pressure_hpa = raw->pressure_hpa;
	proc->temperature_c = raw->temperature_c;
	proc->altitude_m = pressure_to_altitude_m(raw->pressure_hpa);
}

static void preprocess_tof_sample(const raw_tof_sample_t *raw,
		processed_tof_sample_t *proc) {
	proc->timestamp_us = raw->timestamp_us;
	proc->range_status = raw->range_status;
	proc->valid = raw->valid;
	proc->range_m = raw->range_mm / 1000.0f;
}

static void imu_uart_send_raw_line(const LSM6DSR_Axes_t *accel_raw,
		const LSM6DSR_Axes_t *gyro_raw) {
	char buf[128];

	int len = snprintf(buf, sizeof(buf),
			"IMU,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", accel_raw->x / 1000.0f,
			accel_raw->y / 1000.0f, accel_raw->z / 1000.0f,
			gyro_raw->x / 1000.0f, gyro_raw->y / 1000.0f,
			gyro_raw->z / 1000.0f);

	if (len > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

static void imu_uart_send_processed_line(const processed_imu_sample_t *imu) {
	char buf[160];

	int len = snprintf(buf, sizeof(buf),
			"PROC,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", imu->accel_x_mps2,
			imu->accel_y_mps2, imu->accel_z_mps2, imu->gyro_x_rps,
			imu->gyro_y_rps, imu->gyro_z_rps);

	if (len > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

static void baro_uart_send_processed_line(const processed_baro_sample_t *baro) {
	char buf[128];

	int len = snprintf(buf, sizeof(buf), "BAROP,%.2f,%.2f,%.2f\r\n",
			baro->pressure_hpa, baro->temperature_c, baro->altitude_m);

	if (len > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

static void imu_stats_send_line(void) {
	char buf[160];

	int len = snprintf(buf, sizeof(buf), "STAT,%lu,%lu,%lu,%lu,%lu\r\n",
			g_imu_task_loops, g_acc_ready_count, g_gyro_ready_count,
			g_acc_read_count, g_gyro_read_count);

	if (len > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

static void baro_overrun_send_line(void) {
	char buf[128];

	int len = snprintf(buf, sizeof(buf), "BOR,%lu,%lu,%lu\r\n",
			g_baro_status_checks, g_baro_press_overrun_count,
			g_baro_temp_overrun_count);

	if (len > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

static void imu_fifo_send_line(void) {
	char buf[128];

	int len = snprintf(buf, sizeof(buf), "IFIFO,%lu,%u,%lu,%lu\r\n",
			g_imu_fifo_status_checks, g_imu_fifo_last_samples,
			g_imu_fifo_max_samples, g_imu_fifo_overrun_count);

	if (len > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

static void debug_output_send_current_mode(const LSM6DSR_Axes_t *accel,
		const LSM6DSR_Axes_t *gyro, const processed_imu_sample_t *imu_proc,
		const processed_baro_sample_t *baro_proc) {
	switch (g_debug_output_mode) {
	case DEBUG_OUT_RAW_IMU:
		imu_uart_send_raw_line(accel, gyro);
		break;

	case DEBUG_OUT_PROC_IMU:
		imu_uart_send_processed_line(imu_proc);
		break;

	case DEBUG_OUT_PROC_BARO:
		baro_uart_send_processed_line(baro_proc);
		break;

	case DEBUG_OUT_STAT:
		imu_stats_send_line();
		break;

	default:
		break;
	}
}

static void vl53l0x_uart_send_line(const char *text) {
	HAL_UART_Transmit(&huart1, (uint8_t*) text, (uint16_t) strlen(text), 100);
}

static void tof_uart_send_processed_line(const processed_tof_sample_t *tof) {
	char buf[128];

	int len = snprintf(buf, sizeof(buf), "TOF,%lu,%.3f,%u,%u\r\n",
			(unsigned long) tof->timestamp_us, tof->range_m,
			(unsigned int) tof->range_status, (unsigned int) tof->valid);

	if (len > 0) {
		HAL_UART_Transmit(&huart1, (uint8_t*) buf, (uint16_t) len, 100);
	}
}

static void vl53l0x_basic_i2c_test(void) {
	char msg[128];
	uint8_t value = 0;
	HAL_StatusTypeDef status;

	snprintf(msg, sizeof(msg), "\r\n[VL53L0X] basic I2C test start\r\n");
	vl53l0x_uart_send_line(msg);

	status = HAL_I2C_IsDeviceReady(&hi2c2, VL53L0X_I2C_ADDR_8BIT, 3, 100);
	if (status != HAL_OK) {
		snprintf(msg, sizeof(msg), "[VL53L0X] device not ready, status=%d\r\n",
				(int) status);
		vl53l0x_uart_send_line(msg);
		return;
	}

	snprintf(msg, sizeof(msg), "[VL53L0X] device ACKed at 0x%02X\r\n",
	VL53L0X_I2C_ADDR_8BIT);
	vl53l0x_uart_send_line(msg);

	status = vl53l0x_read_u8(0xC0, &value);
	if (status == HAL_OK) {
		snprintf(msg, sizeof(msg), "[VL53L0X] reg 0xC0 = 0x%02X\r\n", value);
	} else {
		snprintf(msg, sizeof(msg), "[VL53L0X] read 0xC0 failed, status=%d\r\n",
				(int) status);
	}
	vl53l0x_uart_send_line(msg);

	status = vl53l0x_read_u8(0xC1, &value);
	if (status == HAL_OK) {
		snprintf(msg, sizeof(msg), "[VL53L0X] reg 0xC1 = 0x%02X\r\n", value);
	} else {
		snprintf(msg, sizeof(msg), "[VL53L0X] read 0xC1 failed, status=%d\r\n",
				(int) status);
	}
	vl53l0x_uart_send_line(msg);

	status = vl53l0x_read_u8(0xC2, &value);
	if (status == HAL_OK) {
		snprintf(msg, sizeof(msg), "[VL53L0X] reg 0xC2 = 0x%02X\r\n", value);
	} else {
		snprintf(msg, sizeof(msg), "[VL53L0X] read 0xC2 failed, status=%d\r\n",
				(int) status);
	}
	vl53l0x_uart_send_line(msg);
}

static void vl53l0x_api_init_test(void) {
	char msg[128];
	VL53L0X_Error status;

	status = vl53l0x_api_init_device();

	if (status == VL53L0X_ERROR_NONE) {
		snprintf(msg, sizeof(msg), "[VL53L0X API] init OK\r\n");
	} else {
		snprintf(msg, sizeof(msg), "[VL53L0X API] init failed, status=%d\r\n",
				(int) status);
	}

	vl53l0x_uart_send_line(msg);
}

static void vl53l0x_single_range_test(void) {
	char msg[160];
	VL53L0X_Error status;
	VL53L0X_RangingMeasurementData_t measurement;

	status = vl53l0x_api_init_device();
	if (status != VL53L0X_ERROR_NONE) {
		snprintf(msg, sizeof(msg), "[VL53L0X RANGE] init failed, status=%d\r\n",
				(int) status);
		vl53l0x_uart_send_line(msg);
		return;
	}

	status = VL53L0X_PerformSingleRangingMeasurement(&g_vl53l0x_dev,
			&measurement);
	if (status != VL53L0X_ERROR_NONE) {
		snprintf(msg, sizeof(msg),
				"[VL53L0X RANGE] measure failed, status=%d\r\n", (int) status);
		vl53l0x_uart_send_line(msg);
		return;
	}

	snprintf(msg, sizeof(msg), "[VL53L0X RANGE] range=%u mm status=%u\r\n",
			(unsigned int) measurement.RangeMilliMeter,
			(unsigned int) measurement.RangeStatus);
	vl53l0x_uart_send_line(msg);
}

static void vl53l0x_acquire_one_sample(void) {
	VL53L0X_Error status;
	VL53L0X_RangingMeasurementData_t measurement;

	status = vl53l0x_api_init_device();
	if (status != VL53L0X_ERROR_NONE) {
		g_raw_tof.timestamp_us = micros();
		g_raw_tof.range_mm = 0;
		g_raw_tof.range_status = (uint8_t) status;
		g_raw_tof.valid = 0;

		preprocess_tof_sample(&g_raw_tof, &g_processed_tof);
		return;
	}

	status = VL53L0X_PerformSingleRangingMeasurement(&g_vl53l0x_dev,
			&measurement);

	g_raw_tof.timestamp_us = micros();
	g_raw_tof.range_mm = measurement.RangeMilliMeter;
	g_raw_tof.range_status = measurement.RangeStatus;
	g_raw_tof.valid =
			(status == VL53L0X_ERROR_NONE && measurement.RangeStatus == 0U) ?
					1U : 0U;

	preprocess_tof_sample(&g_raw_tof, &g_processed_tof);
}

static void calibrate_gyro_bias(void) {
	const uint32_t num_samples = 50;
	uint32_t collected = 0;

	int64_t sum_x = 0;
	int64_t sum_y = 0;
	int64_t sum_z = 0;

	LSM6DSR_Axes_t gyro = { 0 };
	uint8_t gyro_ready = 0;

	HAL_Delay(500);

	while (collected < num_samples) {
		LSM6DSR_GYRO_Get_DRDY_Status(&MotionSensor, &gyro_ready);

		if (gyro_ready) {
			if (LSM6DSR_GYRO_GetAxes(&MotionSensor, &gyro) == LSM6DSR_OK) {
				sum_x += gyro.x;
				sum_y += gyro.y;
				sum_z += gyro.z;
				collected++;
			}
		}

		HAL_Delay(2);
	}

	g_gyro_bias_x_mdps = (float) sum_x / (float) num_samples;
	g_gyro_bias_y_mdps = (float) sum_y / (float) num_samples;
	g_gyro_bias_z_mdps = (float) sum_z / (float) num_samples;
}

static void check_baro_overrun_flags(void) {
	uint8_t status = 0;

	g_baro_status_checks++;

	if (LPS22HH_Read_Reg(&BaroSensor, 0x27, &status) == LPS22HH_OK) {
		if (status & (1U << 5)) {
			g_baro_temp_overrun_count++;
		}

		if (status & (1U << 4)) {
			g_baro_press_overrun_count++;
		}
	}
}

static void check_imu_fifo_status(void) {
	uint16_t num_samples = 0;
	lsm6dsr_reg_t reg;

	g_imu_fifo_status_checks++;

	if (LSM6DSR_FIFO_Get_Num_Samples(&MotionSensor, &num_samples) == LSM6DSR_OK) {
		g_imu_fifo_last_samples = num_samples;

		if (num_samples > g_imu_fifo_max_samples) {
			g_imu_fifo_max_samples = num_samples;
		}
	}

	if (lsm6dsr_read_reg(&(MotionSensor.Ctx), LSM6DSR_FIFO_STATUS2, &reg.byte,
			1) == LSM6DSR_OK) {
		if (reg.fifo_status2.fifo_ovr_ia) {
			g_imu_fifo_overrun_count++;
		}
	}
}

static void drain_imu_fifo_once(void) {
	uint16_t num_samples = 0;
	uint8_t tag = 0;
	uint8_t data[6];
	uint16_t to_drain = 0;
	uint16_t i = 0;

	if (LSM6DSR_FIFO_Get_Num_Samples(&MotionSensor, &num_samples) != LSM6DSR_OK) {
		return;
	}

	if (num_samples == 0) {
		return;
	}

	to_drain = (num_samples > 8U) ? 8U : num_samples;

	for (i = 0; i < to_drain; i++) {
		if (LSM6DSR_FIFO_Get_Tag(&MotionSensor, &tag) != LSM6DSR_OK) {
			break;
		}

		if (LSM6DSR_FIFO_Get_Data(&MotionSensor, data) != LSM6DSR_OK) {
			break;
		}

		g_imu_fifo_drained_samples++;
	}
}

int32_t SensorManager_Init(void) {
	assert(MX_LSM6DSR_Init() == LSM6DSR_OK);
	assert(MX_LSM6DSR_FIFO_Test_Init() == LSM6DSR_OK);
	assert(MX_LPS22HH_Init() == LPS22HH_OK);

	calibrate_gyro_bias();

	g_imu_task_loops = 0;
	g_acc_ready_count = 0;
	g_gyro_ready_count = 0;
	g_acc_read_count = 0;
	g_gyro_read_count = 0;

	return 0;
}

void SensorManager_RunOnce(void) {
	LSM6DSR_Axes_t accel = { 0 };
	LSM6DSR_Axes_t gyro = { 0 };
	float pressure_hpa = 0.0f;
	float temperature_c = 0.0f;

	uint8_t acc_ready = 0;
	uint8_t gyro_ready = 0;
	uint8_t press_ready = 0;
	uint8_t temp_ready = 0;

	g_imu_task_loops++;

	LSM6DSR_ACC_Get_DRDY_Status(&MotionSensor, &acc_ready);
	LSM6DSR_GYRO_Get_DRDY_Status(&MotionSensor, &gyro_ready);
	check_imu_fifo_status();
	drain_imu_fifo_once();

	if (acc_ready) {
		g_acc_ready_count++;
	}

	if (gyro_ready) {
		g_gyro_ready_count++;
	}

	if (acc_ready) {
		LSM6DSR_ACC_GetAxes(&MotionSensor, &accel);
		g_acc_read_count++;
	}

	if (gyro_ready) {
		LSM6DSR_GYRO_GetAxes(&MotionSensor, &gyro);
		g_gyro_read_count++;
	}

	g_raw_imu.timestamp_us = micros();
	g_raw_imu.accel_x_mg = accel.x;
	g_raw_imu.accel_y_mg = accel.y;
	g_raw_imu.accel_z_mg = accel.z;
	g_raw_imu.gyro_x_mdps = gyro.x;
	g_raw_imu.gyro_y_mdps = gyro.y;
	g_raw_imu.gyro_z_mdps = gyro.z;
	g_raw_imu.accel_ready = acc_ready;
	g_raw_imu.gyro_ready = gyro_ready;

	osMutexAcquire(IMUDataMutexID, osWaitForever);
	preprocess_imu_sample(&g_raw_imu, &g_processed_imu);
	osMutexRelease(IMUDataMutexID);

	LPS22HH_PRESS_Get_DRDY_Status(&BaroSensor, &press_ready);
	LPS22HH_TEMP_Get_DRDY_Status(&BaroSensor, &temp_ready);
	check_baro_overrun_flags();

	if (press_ready) {
		LPS22HH_PRESS_GetPressure(&BaroSensor, &pressure_hpa);
	}

	if (temp_ready) {
		LPS22HH_TEMP_GetTemperature(&BaroSensor, &temperature_c);
	}

	g_raw_baro.timestamp_us = micros();
	g_raw_baro.pressure_hpa = pressure_hpa;
	g_raw_baro.temperature_c = temperature_c;
	g_raw_baro.pressure_ready = press_ready;
	g_raw_baro.temperature_ready = temp_ready;

	osMutexAcquire(altitudeDataMutexID, osWaitForever);
	preprocess_baro_sample(&g_raw_baro, &g_processed_baro);
	osMutexRelease(altitudeDataMutexID);

	/* Keeping all debug TX behavior exactly as it is now: still commented */
	/* imu_uart_send_raw_line(&accel, &gyro); */
	/* imu_uart_send_processed_line(&g_processed_imu); */
	/* baro_uart_send_processed_line(&g_processed_baro); */
	/* debug_output_send_current_mode(&accel, &gyro, &g_processed_imu, &g_processed_baro); */
	/* baro_overrun_send_line(); */
	/* imu_fifo_send_line(); */

	/* Keep these unused helper references in file so compiler does not warn if needed later */
	(void) g_debug_output_mode;
	(void) g_processed_tof;
	(void) vl53l0x_basic_i2c_test;
	(void) vl53l0x_api_init_test;
	(void) vl53l0x_single_range_test;
	(void) vl53l0x_acquire_one_sample;
	(void) tof_uart_send_processed_line;
	(void) debug_output_send_current_mode;
	(void) baro_overrun_send_line;
	(void) imu_fifo_send_line;
}
