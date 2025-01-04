/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
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

#include "system_interface.h"

/* Standard includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* IMU drivers */
#include "inv_imu_defs.h" /* For error codes */

#define INV_MSG_LEVEL_TOP INV_MSG_LEVEL_OFF

#include "pxt.h"

#if MICROBIT_CODAL
#define I2C_BUFFER_TYPE uint8_t *
#else
#define I2C_BUFFER_TYPE char *
#endif

enum class DigitalPin
{
};

//%
namespace icm42670p
{

	static int slave_address = 0b1101000; // 7-bit slave address
	static DigitalPin int1_pin = (DigitalPin)MICROBIT_ID_IO_P16;
	static void (*int1_cb)(void *context, unsigned int_num);

	//%
	void set_slave_address(int addr)
	{
		icm42670p::slave_address = addr;
	}

	//%
	void set_int1_pin(DigitalPin pin)
	{
		icm42670p::int1_pin = pin;
	}

} // namespace icm42670p

/*
 * I/O for IMU device
 */
int si_io_imu_init(uint32_t serif_type)
{
	(void)serif_type;
	return 0;
}

int si_io_imu_read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	const int addr = icm42670p::slave_address << 1;
	if (MICROBIT_OK != pxt::uBit.i2c.write(addr, (I2C_BUFFER_TYPE)&reg, 1, true))
	{
		return INV_ERROR;
	}
	if (MICROBIT_OK != pxt::uBit.i2c.read(addr, (I2C_BUFFER_TYPE)buf, len, false))
	{
		return INV_ERROR;
	}
	return 0;
}

int si_io_imu_write_reg(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	const int addr = icm42670p::slave_address << 1;

	pxt::Buffer buf_obj = pins::createBuffer(1 + len);
	pxt::registerGCObj(buf_obj); // make sure buffer is pinned, while we use data

	I2C_BUFFER_TYPE buf1 = (I2C_BUFFER_TYPE)buf_obj->data;
	memcpy(buf1, &reg, 1);
	memcpy(buf1 + 1, buf, len);

	int res = MICROBIT_OK == pxt::uBit.i2c.write(addr, buf1, 1 + len, false) ? 0 : INV_ERROR;

	pxt::unregisterGCObj(buf_obj);

	return res;
}

/*
 * Timers
 */
int si_init_timers()
{
	return 0;
}

void si_sleep_us(uint32_t us)
{
	if (us >= 1000)
	{
		uint32_t ms = us / 1000;
		us %= 1000;

		// https://github.com/microsoft/pxt-microbit/blob/ba2e8e95b329dd947e6e11ccf5d230ad43cbb2c0/libs/core/basic.cpp#L101
		fiber_sleep(ms);
	}

	if (us > 0)
	{
		// https://github.com/microsoft/pxt-microbit/blob/ba2e8e95b329dd947e6e11ccf5d230ad43cbb2c0/libs/core/control.cpp#L288
		pxt::sleep_us(us);
	}
}

uint64_t si_get_time_us()
{
	// https://github.com/microsoft/pxt-microbit/blob/ba2e8e95b329dd947e6e11ccf5d230ad43cbb2c0/libs/core/control.cpp#L244
	// return system_timer_current_time_us() & 0x3fffffff;
	return system_timer_current_time_us();
}

int si_start_periodic_timer(const uint32_t period_us, void callback(void *context), int *ch)
{
	(void)period_us;
	(void)ch;
	return 0;
}

int si_stop_periodic_timer(int ch)
{
	(void)ch;
	return 0;
}

static void int1_pulse_cb(MicroBitEvent e)
{
	if (icm42670p::int1_cb)
		icm42670p::int1_cb((void *)0, SI_GPIO_INT1);
}

/*
 * GPIO
 */
int si_init_gpio_int(unsigned int_num, void (*int_cb)(void *context, unsigned int_num))
{
	if (SI_GPIO_INT1 != int_num)
	{
		return INV_ERROR_BAD_ARG;
	}

	const int id = (int)icm42670p::int1_pin;
	MicroBitPin *pin = pxt::getPin(id);
	if (!pin)
		return INV_ERROR;

	icm42670p::int1_cb = int_cb;

	pin->eventOn(MICROBIT_PIN_EVENT_ON_PULSE);

	pxt::uBit.messageBus.listen(id, MICROBIT_PIN_EVT_PULSE_HI, int1_pulse_cb);

	return 0;
}

/*
 * Common
 */

void si_disable_irq()
{
}

void si_enable_irq()
{
}

/*
 * Error codes
 */
int si_print_error_if_any(int rc)
{
	if (rc < 0)
	{
		switch (rc)
		{
		case INV_ERROR:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Unspecified error (%d)", rc);
			break;
		case INV_ERROR_NIMPL:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Function not implemented for given arguments (%d)", rc);
			break;
		case INV_ERROR_TRANSPORT:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Error occurred at transport level (%d)", rc);
			break;
		case INV_ERROR_TIMEOUT:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Action did not complete in the expected time window (%d)",
					rc);
			break;
		case INV_ERROR_SIZE:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Invalid argument's size provided (%d)", rc);
			break;
		case INV_ERROR_BAD_ARG:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Invalid argument provided (%d)", rc);
			break;
		case INV_ERROR_UNEXPECTED:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Something unexpected happened (%d)", rc);
			break;
		default:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Unknown error (%d)", rc);
			break;
		}

		return rc;
	}

	return 0;
}

/*
 * Message
 */
void inv_msg(int level, const char *str, ...)
{
#if INV_MSG_LEVEL_TOP != INV_MSG_LEVEL_OFF
	if (level <= INV_MSG_LEVEL_OFF || level >= INV_MSG_LEVEL_MAX || level >= INV_MSG_LEVEL_TOP)
	{
		return;
	}

	va_list args;
	va_start(args, str);
	pxt::debuglog(str, args);
	va_end(args);

#endif
}
