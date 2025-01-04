/*
 *
 * Copyright (c) [2018] by InvenSense, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "pxt.h"

enum EventBusSource
{
	//% blockIdentity="control.eventSourceId"
	ICM42670P_DEVICE_ID = 74,
};

enum EventBusValue
{
	//% blockIdentity="control.eventValueId"
	ICM42670P_EVT_DATA_UPDATED = 1,
};

/* Driver */
#include "inv_imu_driver.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * This example showcases how to configure IMU to stream accel and gyro data.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_I2C

//%
namespace icm42670p
{

	/* Static variables */
	static inv_imu_device_t imu_dev;		 /* Driver structure */
	static volatile int int1_flag;			 /* Flag set when INT1 is asserted */
	static volatile uint64_t int1_timestamp; /* Timestamp when INT1 is asserted */

	/* Static variables for command interface */
	// static uint8_t print_si = 0;  /* Indicates if data should be printed in SI */
	// static uint8_t print_lsb = 1; /* Indicates if data should be printed in LSB */
	static uint8_t fifo_en = 1;	 /* Indicates if data are read from FIFO (1) or registers (0) */
	static uint8_t hires_en = 1; /* Indicates if highres is enabled */
	static uint8_t use_ln = 1;	 /* Indicates if power mode is low noise (1) or low power (0) */

	static volatile int accel_raw[3];
	static volatile int gyro_raw[3];
	static volatile int16_t temp_raw;

	/* Static functions definition */
	static int setup_imu();
	static int configure_fifo();
	static int configure_hires();
	static int configure_power_mode();
	static void int_cb(void *context, unsigned int int_num);
	static void sensor_event_cb(inv_imu_sensor_event_t *event);

	//%
	int main()
	{
		int rc = 0;

		/* Configure GPIO to call `int_cb` when INT1 fires. */
		rc |= si_init_gpio_int(SI_GPIO_INT1, int_cb);
		SI_CHECK_RC(rc);
		if (rc)
		{
			return rc;
		}

		INV_MSG(INV_MSG_LEVEL_INFO, "###");
		INV_MSG(INV_MSG_LEVEL_INFO, "### Example Raw");
		INV_MSG(INV_MSG_LEVEL_INFO, "###");

		/* Reset commands interface states */
		// print_si = 1;
		// print_lsb = 0;
		// fifo_en = 1;
		// hires_en = 1;
		// use_ln = 1;

		rc |= setup_imu();
		SI_CHECK_RC(rc);
		if (rc)
		{
			return rc;
		}

		/* Reset timestamp and interrupt flag */
		int1_flag = 0;
		int1_timestamp = 0;

		do
		{
			/* Poll device for data */
			if (int1_flag)
			{
				si_disable_irq();
				/* Clear interrupt flag */
				int1_flag = 0;
				si_enable_irq();

				if (fifo_en)
					rc |= inv_imu_get_data_from_fifo(&imu_dev);
				else
					rc |= inv_imu_get_data_from_registers(&imu_dev);

				SI_CHECK_RC(rc);
				rc = 0; /* reset `rc` (contains the number of packet read if above check is passing) */
			}

			si_sleep_us(1000);

		} while (rc == 0);

		return rc;
	}

	//%
	int set_fifo_enabled(bool enabled)
	{
		int rc = 0;
		enabled = !!enabled;
		if (icm42670p::fifo_en != enabled)
		{
			icm42670p::fifo_en = enabled;
			rc |= configure_fifo();
			rc |= configure_hires();
		}
		return rc;
	}

	//%
	int set_highres_enabled(bool enabled)
	{
		int rc = 0;
		enabled = !!enabled;
		if (icm42670p::hires_en != enabled)
		{
			icm42670p::hires_en = enabled;
			if (!fifo_en)
				INV_MSG(INV_MSG_LEVEL_INFO, "Warning: highres mode only apply if FIFO is enabled.");
			rc |= configure_hires();
		}
		return rc;
	}

	//%
	int set_ln_enabled(bool enabled)
	{
		int rc = 0;
		enabled = !!enabled;
		if (icm42670p::use_ln != enabled)
		{
			icm42670p::use_ln = enabled;
			rc |= configure_power_mode();
		}
		return rc;
	}

	//%
	Buffer get_accel_data(bool raw)
	{
		if (raw)
		{
			auto buf = pins::createBuffer(3 * sizeof(int));
			memcpy(buf->data, (void *)icm42670p::accel_raw, buf->length);
			return buf;
		}

		// Convert to g ---

#if INV_IMU_HFSR_SUPPORTED
		uint16_t accel_fsr_g = fifo_en && hires_en ? 32 : 4;
#else
		uint16_t accel_fsr_g = fifo_en && hires_en ? 16 : 4;
#endif
		const int max_lsb = fifo_en && hires_en ? 524287 : 32768;

		auto buf = pins::createBuffer(3 * sizeof(float));
		float *buf1 = (float *)buf->data;

		int *accel_raw = (int *)icm42670p::accel_raw;
		buf1[0] = (float)(accel_raw[0] * accel_fsr_g) / (float)max_lsb;
		buf1[1] = (float)(accel_raw[1] * accel_fsr_g) / (float)max_lsb;
		buf1[2] = (float)(accel_raw[2] * accel_fsr_g) / (float)max_lsb;

		return buf;
	}

	//%
	Buffer get_gyro_data(bool raw)
	{
		if (raw)
		{
			auto buf = pins::createBuffer(3 * sizeof(int));
			memcpy(buf->data, (void *)icm42670p::gyro_raw, buf->length);
			return buf;
		}

		// Convert to dps ---

#if INV_IMU_HFSR_SUPPORTED
		uint16_t accel_fsr_g = fifo_en && hires_en ? 32 : 4;
		uint16_t gyro_fsr_dps = fifo_en && hires_en ? 4000 : 2000;
#else
		uint16_t accel_fsr_g = fifo_en && hires_en ? 16 : 4;
		uint16_t gyro_fsr_dps = 2000;
#endif
		const int max_lsb = fifo_en && hires_en ? 524287 : 32768;

		auto buf = pins::createBuffer(3 * sizeof(float));
		float *buf1 = (float *)buf->data;

		int *gyro_raw = (int *)icm42670p::gyro_raw;
		buf1[0] = (float)(gyro_raw[0] * gyro_fsr_dps) / (float)max_lsb;
		buf1[1] = (float)(gyro_raw[1] * gyro_fsr_dps) / (float)max_lsb;
		buf1[2] = (float)(gyro_raw[2] * gyro_fsr_dps) / (float)max_lsb;

		return buf;
	}

	//%
	int get_temp_data(bool raw)
	{
		if (raw)
			return icm42670p::temp_raw;

		if (hires_en || !fifo_en)
			return 25 + ((float)icm42670p::temp_raw / 128);
		else
			return 25 + ((float)icm42670p::temp_raw / 2);
	}

	static int setup_imu()
	{
		int rc = 0;
		inv_imu_serif_t imu_serif;
		uint8_t whoami;
		inv_imu_int1_pin_config_t int1_pin_config;

		/* Initialize serial interface between MCU and IMU */
		imu_serif.context = 0; /* no need */
		imu_serif.read_reg = si_io_imu_read_reg;
		imu_serif.write_reg = si_io_imu_write_reg;
		imu_serif.max_read = 1024 * 32;	 /* maximum number of bytes allowed per serial read */
		imu_serif.max_write = 1024 * 32; /* maximum number of bytes allowed per serial write */
		imu_serif.serif_type = SERIF_TYPE;

		/* Init device */
		rc |= inv_imu_init(&imu_dev, &imu_serif, sensor_event_cb);
		SI_CHECK_RC(rc);
		if (rc)
		{
			return rc;
		}

#if SERIF_TYPE == UI_SPI4
		/* Configure slew-rate to 19 ns (required when using EVB) */
		rc |= inv_imu_set_spi_slew_rate(&imu_dev, DRIVE_CONFIG3_SPI_SLEW_RATE_MAX_19_NS);
		SI_CHECK_RC(rc);
		if (rc)
		{
			return rc;
		}
#endif

		/* Check WHOAMI */
		rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
		SI_CHECK_RC(rc);
		if (rc)
		{
			return rc;
		}

		if (whoami != INV_IMU_WHOAMI)
		{
			INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
			INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
			INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
			return INV_ERROR;
		}

		/*
		 * Configure interrupts pins
		 * - Polarity High
		 * - Pulse mode
		 * - Push-Pull drive
		 */
		int1_pin_config.int_polarity = INT_CONFIG_INT1_POLARITY_HIGH;
		int1_pin_config.int_mode = INT_CONFIG_INT1_MODE_PULSED;
		int1_pin_config.int_drive = INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
		rc |= inv_imu_set_pin_config_int1(&imu_dev, &int1_pin_config);

		/* Configure FSR (doesn't apply if FIFO is used in highres mode) */
		rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_FS_SEL_4g);
		rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_FS_SEL_2000dps);

		/* Configure ODR */
		rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ODR_50_HZ);
		rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_ODR_50_HZ);

		/* Variable configuration */
		rc |= configure_fifo();
		rc |= configure_hires();
		rc |= configure_power_mode();

		return rc;
	}

	static int configure_fifo()
	{
		int rc = 0;
		inv_imu_interrupt_parameter_t int1_config; // = {(inv_imu_interrupt_value)0};
		memset(&int1_config, 0, sizeof(inv_imu_interrupt_parameter_t));

		rc |= inv_imu_configure_fifo(&imu_dev, fifo_en ? INV_IMU_FIFO_ENABLED : INV_IMU_FIFO_DISABLED);

		/* Configure interrupts sources */
		if (fifo_en)
			int1_config.INV_FIFO_THS = INV_IMU_ENABLE;
		else
			int1_config.INV_UI_DRDY = INV_IMU_ENABLE;
		rc |= inv_imu_set_config_int1(&imu_dev, &int1_config);

		return rc;
	}

	static int configure_hires()
	{
		int rc = 0;

		if (hires_en)
			rc |= inv_imu_enable_high_resolution_fifo(&imu_dev);
		else
			rc |= inv_imu_disable_high_resolution_fifo(&imu_dev);

		return rc;
	}

	static int configure_power_mode()
	{
		int rc = 0;

		if (use_ln)
			rc |= inv_imu_enable_accel_low_noise_mode(&imu_dev);
		else
			rc |= inv_imu_enable_accel_low_power_mode(&imu_dev);

		rc |= inv_imu_enable_gyro_low_noise_mode(&imu_dev);

		return rc;
	}

	/* IMU interrupt handler. */
	static void int_cb(void *context, unsigned int int_num)
	{
		(void)context;

		if (int_num == SI_GPIO_INT1)
		{
			int1_timestamp = si_get_time_us();
			int1_flag = 1;
		}
	}

	static void sensor_event_cb(inv_imu_sensor_event_t *event)
	{
		uint64_t int_timestamp = 0;
		// int accel_raw[3];
		// int gyro_raw[3];
		// char accel_str[40];
		// char gyro_str[40];
		// char temp_str[20];
		// char fifo_time_str[30];
		volatile int *accel_raw = icm42670p::accel_raw;
		volatile int *gyro_raw = icm42670p::gyro_raw;
		volatile int16_t *temp_raw = &icm42670p::temp_raw;

		si_disable_irq();
		int_timestamp = int1_timestamp;
		si_enable_irq();

		if (fifo_en)
		{
			uint64_t fifo_timestamp;
			static uint64_t last_fifo_timestamp = 0;
			static uint32_t rollover_num = 0;

			/* Handle rollover */
			if (last_fifo_timestamp > event->timestamp_fsync)
				rollover_num++;
			last_fifo_timestamp = event->timestamp_fsync;

			/* Compute timestamp in us */
			if (last_fifo_timestamp == 0 && rollover_num == 0)
			{
				fifo_timestamp = int_timestamp;
			}
			else
			{
				fifo_timestamp = event->timestamp_fsync + rollover_num * UINT16_MAX;
				fifo_timestamp *= inv_imu_get_timestamp_resolution_us(&imu_dev);
			}

			// snprintf(fifo_time_str, 30, "FIFO Time: %5llu us", fifo_timestamp);

			if (hires_en)
			{
				accel_raw[0] = ((int32_t)event->accel[0] << 4) | event->accel_high_res[0];
				accel_raw[1] = ((int32_t)event->accel[1] << 4) | event->accel_high_res[1];
				accel_raw[2] = ((int32_t)event->accel[2] << 4) | event->accel_high_res[2];
				gyro_raw[0] = ((int32_t)event->gyro[0] << 4) | event->gyro_high_res[0];
				gyro_raw[1] = ((int32_t)event->gyro[1] << 4) | event->gyro_high_res[1];
				gyro_raw[2] = ((int32_t)event->gyro[2] << 4) | event->gyro_high_res[2];
			}
			else
			{
				accel_raw[0] = event->accel[0];
				accel_raw[1] = event->accel[1];
				accel_raw[2] = event->accel[2];
				gyro_raw[0] = event->gyro[0];
				gyro_raw[1] = event->gyro[1];
				gyro_raw[2] = event->gyro[2];
			}
		}
		else
		{
			/* No timestamp info if not using FIFO */
			// snprintf(fifo_time_str, 30, " ");

			accel_raw[0] = event->accel[0];
			accel_raw[1] = event->accel[1];
			accel_raw[2] = event->accel[2];

			gyro_raw[0] = event->gyro[0];
			gyro_raw[1] = event->gyro[1];
			gyro_raw[2] = event->gyro[2];

			/* Force sensor_mask so it gets displayed below */
			event->sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
			event->sensor_mask |= (1 << INV_SENSOR_ACCEL);
			event->sensor_mask |= (1 << INV_SENSOR_GYRO);
		}

		*temp_raw = event->temperature;

		// Raise event
		MicroBitEvent e(ICM42670P_DEVICE_ID, ICM42670P_EVT_DATA_UPDATED, CREATE_AND_FIRE);
		(void)e;
	}

} // namespace icm42670p
