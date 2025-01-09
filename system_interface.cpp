/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "pxt.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "system_interface.h"

#include "Message.h"
#include "InvError.h"

#if MICROBIT_CODAL
#define I2C_BUFFER_TYPE uint8_t *
#else
#define I2C_BUFFER_TYPE char *
#endif

enum class DigitalPin
{
};

static void int1_pulse_cb(MicroBitEvent e);

//%
namespace icm42670p
{

    /* I2C number and slave address for INV device */
    static int ICM_I2C_ADDR = 0b1101000;

    static DigitalPin INT1_PIN = (DigitalPin)MICROBIT_ID_IO_P16;
    static void (*INT1_CB)(void *context, unsigned int_num);

    //%
    void set_slave_address(int addr)
    {
        icm42670p::ICM_I2C_ADDR = addr;
    }

    //%
    void set_int1_pin(DigitalPin pin)
    {
        icm42670p::INT1_PIN = pin;
    }

} // namespace icm42670p

void config_uart()
{
    // Config uart for log
}

int uart_puts(const char *s, unsigned short l)
{
    if (l > 0)
    {
        pxt::uBit.serial.send((uint8_t *)s, (int)l);
    }
    return 0;
}

/*----------------------------------------------------*/
/* Low-level serial interface function implementation */
/*----------------------------------------------------*/

void inv_io_hal_board_init(void)
{
    // Init board
}

int inv_io_hal_init()
{
    // Init I2C interface
    return 0;
}

int inv_io_hal_read_reg(struct inv_imu_serif *serif, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    const int addr = icm42670p::ICM_I2C_ADDR << 1;
    if (MICROBIT_OK != pxt::uBit.i2c.write(addr, (I2C_BUFFER_TYPE)&reg, 1, true))
    {
        return INV_ERROR;
    }
    if (MICROBIT_OK != pxt::uBit.i2c.read(addr, (I2C_BUFFER_TYPE)rbuffer, rlen, false))
    {
        return INV_ERROR;
    }
    return 0;
}

int inv_io_hal_write_reg(struct inv_imu_serif *serif, uint8_t reg, const uint8_t *wbuffer,
                         uint32_t wlen)
{
    const int addr = icm42670p::ICM_I2C_ADDR << 1;

    pxt::Buffer buf_obj = pins::createBuffer(1 + wlen);
    pxt::registerGCObj(buf_obj); // make sure buffer is pinned, while we use data

    I2C_BUFFER_TYPE buf1 = (I2C_BUFFER_TYPE)buf_obj->data;
    memcpy(buf1, &reg, 1);
    memcpy(buf1 + 1, wbuffer, wlen);

    int res = MICROBIT_OK == pxt::uBit.i2c.write(addr, buf1, 1 + wlen, false) ? 0 : INV_ERROR;

    pxt::unregisterGCObj(buf_obj);

    return res;
}

void inv_gpio_sensor_irq_init(unsigned pin_num,
                              void (*interrupt_cb)(void *context, unsigned int_num), void *context)
{
    if (INV_GPIO_INT1 != pin_num)
    {
        return;
    }

    const int id = (int)icm42670p::INT1_PIN;
    MicroBitPin *pin = pxt::getPin(id);
    if (!pin)
    {
        INV_MSG(INV_MSG_LEVEL_ERROR, "Invalid INT1 pin %d", id);
        return;
    }

    icm42670p::INT1_CB = interrupt_cb;

    pin->eventOn(MICROBIT_PIN_EVENT_ON_PULSE);

    pxt::uBit.messageBus.listen(id, MICROBIT_PIN_EVT_PULSE_HI, int1_pulse_cb);
}

static void int1_pulse_cb(MicroBitEvent e)
{
    if (icm42670p::INT1_CB)
        icm42670p::INT1_CB((void *)0, INV_GPIO_INT1);
}

// void inv_disable_irq(void) {}
// void inv_enable_irq(void) {}

void inv_delay_us(uint32_t us)
{
    if (us >= 1000)
    {
        uint32_t ms = us / 1000;
        us %= 1000;
        inv_delay_ms(ms);
    }

    if (us > 0)
    {
        // https://github.com/microsoft/pxt-microbit/blob/ba2e8e95b329dd947e6e11ccf5d230ad43cbb2c0/libs/core/control.cpp#L288
        pxt::sleep_us(us);
    }
}

void inv_delay_ms(uint32_t ms)
{
    // https://github.com/microsoft/pxt-microbit/blob/ba2e8e95b329dd947e6e11ccf5d230ad43cbb2c0/libs/core/basic.cpp#L101
    fiber_sleep(ms);
}

uint64_t inv_get_time_us()
{
    // https://github.com/microsoft/pxt-microbit/blob/ba2e8e95b329dd947e6e11ccf5d230ad43cbb2c0/libs/core/control.cpp#L244
    // return system_timer_current_time_us() & 0x3fffffff;
    return system_timer_current_time_us();
}
